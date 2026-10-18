// AVFoundation's audio types are not marked Sendable yet; they are only touched
// on the main actor here and handed to the speech analyzer, which Apple designed for.
@preconcurrency import AVFoundation
import Speech

/// Runs Apple's on-device speech-to-text (SpeechAnalyzer, iOS 26) on 16 kHz mono
/// Int16 audio pushed in with `feed(_:)`. Nothing is sent to a server.
@MainActor
final class LiveTranscriber {
    enum Failure: LocalizedError {
        case unavailable
        case localeUnsupported
        case noAudioFormat
        case converterUnavailable

        var errorDescription: String? {
            switch self {
            case .unavailable: "On-device speech recognition is not available on this iPhone."
            case .localeUnsupported: "Your iPhone's language is not supported for on-device transcription."
            case .noAudioFormat: "The speech model reported no usable audio format."
            case .converterUnavailable: "Could not convert the glasses' audio to the speech model's format."
            }
        }
    }

    static let sourceFormat = AVAudioFormat(
        commonFormat: .pcmFormatInt16, sampleRate: 16_000, channels: 1, interleaved: true)!

    var onResult: (@MainActor (_ text: String, _ isFinal: Bool) -> Void)?
    var onError: (@MainActor (Error) -> Void)?

    private var analyzer: SpeechAnalyzer?
    private var input: AsyncStream<AnalyzerInput>.Continuation?
    private var resultsTask: Task<Void, Never>?
    private var analyzerFormat: AVAudioFormat?
    private var converter: AVAudioConverter?

    func start(status: @MainActor (String) -> Void) async throws {
        guard SpeechTranscriber.isAvailable else { throw Failure.unavailable }

        let authorization = await Self.requestSpeechAuthorization()
        if authorization != .authorized {
            // SpeechAnalyzer runs on-device and may not need this; the request is a safety net.
            print("Speech recognition permission is \(authorization.rawValue); trying anyway.")
        }

        guard let locale = await SpeechTranscriber.supportedLocale(equivalentTo: Locale.current) else {
            throw Failure.localeUnsupported
        }
        let transcriber = SpeechTranscriber(locale: locale, preset: .progressiveTranscription)

        // First launch only: the model for this language is downloaded once.
        if let request = try await AssetInventory.assetInstallationRequest(supporting: [transcriber]) {
            status("Downloading speech model…")
            try await request.downloadAndInstall()
        }

        guard let format = await SpeechAnalyzer.bestAvailableAudioFormat(
            compatibleWith: [transcriber], considering: Self.sourceFormat)
        else { throw Failure.noAudioFormat }
        if format != Self.sourceFormat {
            guard let converter = AVAudioConverter(from: Self.sourceFormat, to: format) else {
                throw Failure.converterUnavailable
            }
            converter.primeMethod = .none
            self.converter = converter
        }
        analyzerFormat = format

        let analyzer = SpeechAnalyzer(modules: [transcriber])
        try await analyzer.prepareToAnalyze(in: format)
        self.analyzer = analyzer

        resultsTask = Task { [weak self] in
            do {
                for try await result in transcriber.results {
                    self?.onResult?(String(result.text.characters), result.isFinal)
                }
            } catch {
                self?.onError?(error)
            }
        }

        let (stream, continuation) = AsyncStream<AnalyzerInput>.makeStream(bufferingPolicy: .bufferingNewest(100))
        input = continuation
        try await analyzer.start(inputSequence: stream)
        status("Listening")
    }

    func feed(_ samples: [Int16]) {
        guard let input, let analyzerFormat, !samples.isEmpty,
              let buffer = AVAudioPCMBuffer(
                pcmFormat: Self.sourceFormat, frameCapacity: AVAudioFrameCount(samples.count))
        else { return }

        buffer.frameLength = AVAudioFrameCount(samples.count)
        samples.withUnsafeBufferPointer { src in
            buffer.int16ChannelData![0].update(from: src.baseAddress!, count: samples.count)
        }

        if let converter {
            guard let converted = convert(buffer, with: converter, to: analyzerFormat) else { return }
            input.yield(AnalyzerInput(buffer: converted))
        } else {
            input.yield(AnalyzerInput(buffer: buffer))
        }
    }

    func stop() async {
        input?.finish()
        input = nil
        try? await analyzer?.finalizeAndFinishThroughEndOfInput()
        resultsTask?.cancel()
        resultsTask = nil
        analyzer = nil
    }

    // Streaming conversion: `.noDataNow` keeps the resampler's state between calls.
    private func convert(_ buffer: AVAudioPCMBuffer, with converter: AVAudioConverter, to format: AVAudioFormat)
        -> AVAudioPCMBuffer? {
        let ratio = format.sampleRate / buffer.format.sampleRate
        let capacity = AVAudioFrameCount((Double(buffer.frameLength) * ratio).rounded(.up)) + 1
        guard let out = AVAudioPCMBuffer(pcmFormat: format, frameCapacity: capacity) else { return nil }

        var consumed = false
        var error: NSError?
        let status = converter.convert(to: out, error: &error) { _, inputStatus in
            if consumed {
                inputStatus.pointee = .noDataNow
                return nil
            }
            consumed = true
            inputStatus.pointee = .haveData
            return buffer
        }
        if status == .error {
            print("Audio conversion failed: \(String(describing: error))")
            return nil
        }
        return out
    }

    // Nonisolated so the system's callback, which arrives off the main thread,
    // is not treated as main-actor code.
    nonisolated private static func requestSpeechAuthorization() async -> SFSpeechRecognizerAuthorizationStatus {
        await withCheckedContinuation { continuation in
            SFSpeechRecognizer.requestAuthorization { continuation.resume(returning: $0) }
        }
    }
}
