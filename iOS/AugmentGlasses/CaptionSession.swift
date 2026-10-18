import Foundation
import Observation

/// Wires the glasses' audio into the transcriber and sends the captions back.
@MainActor
@Observable
final class CaptionSession {
    let link = GlassesLink()

    private(set) var finalizedText = ""
    private(set) var volatileText = ""
    private(set) var transcriberStatus = "Starting…"

    /// Older text is dropped so memory and the on-screen transcript stay small.
    private static let historyLimit = 4000

    @ObservationIgnored private let transcriber = LiveTranscriber()
    @ObservationIgnored private var started = false

    var displayText: String { Self.join(finalizedText, volatileText) }

    func start() async {
        guard !started else { return }
        started = true

        link.onAudio = { [weak self] samples in self?.transcriber.feed(samples) }
        transcriber.onResult = { [weak self] text, isFinal in self?.handleResult(text, isFinal: isFinal) }
        transcriber.onError = { [weak self] error in
            self?.transcriberStatus = "Transcription stopped: \(error.localizedDescription)"
        }

        do {
            try await transcriber.start { [weak self] status in self?.transcriberStatus = status }
        } catch {
            transcriberStatus = error.localizedDescription
        }
    }

    func clear() {
        finalizedText = ""
        volatileText = ""
        link.sendCaption("", isFinal: true)
    }

    private func handleResult(_ text: String, isFinal: Bool) {
        if isFinal {
            finalizedText = Self.join(finalizedText, text)
            if finalizedText.count > Self.historyLimit {
                finalizedText = String(finalizedText.suffix(Self.historyLimit))
            }
            volatileText = ""
        } else {
            volatileText = text
        }
        link.sendCaption(displayText, isFinal: isFinal)
    }

    private static func join(_ a: String, _ b: String) -> String {
        let a = a.trimmingCharacters(in: .whitespaces)
        let b = b.trimmingCharacters(in: .whitespaces)
        if a.isEmpty { return b }
        if b.isEmpty { return a }
        return a + " " + b
    }
}
