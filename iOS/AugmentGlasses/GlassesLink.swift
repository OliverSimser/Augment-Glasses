@preconcurrency import CoreBluetooth
import Foundation
import Observation

/// BLE link to the glasses: finds and connects to the ESP32, decodes the audio it
/// streams, and writes caption text back. Must match speech_text_screen.ino.
@MainActor
@Observable
final class GlassesLink: NSObject {
    static let serviceUUID = CBUUID(string: "4A8E0001-6C2B-4E5F-9A3D-1C7B2F0E8D10")
    static let audioUUID = CBUUID(string: "4A8E0002-6C2B-4E5F-9A3D-1C7B2F0E8D10")
    static let captionUUID = CBUUID(string: "4A8E0003-6C2B-4E5F-9A3D-1C7B2F0E8D10")

    /// About three lines on a small display; also keeps the ESP32's Serial log readable.
    static let captionMaxBytes = 180
    private static let captionMinInterval: TimeInterval = 0.1
    private static let headerBytes = 4
    /// Longer gaps are not padded with silence; the speech model copes with a jump.
    private static let maxLostPacketsToPad = 10

    enum State: String {
        case bluetoothOff = "Bluetooth is off"
        case unauthorized = "Bluetooth permission denied"
        case searching = "Searching for glasses…"
        case connecting = "Connecting…"
        case connected = "Connected"
        case streaming = "Receiving audio"
    }

    private(set) var state: State = .bluetoothOff
    private(set) var packetsPerSecond = 0
    private(set) var lostPackets = 0
    private(set) var mtu = 0

    /// Decoded 16 kHz mono samples, in order, with lost packets padded with silence.
    @ObservationIgnored var onAudio: (@MainActor ([Int16]) -> Void)?

    @ObservationIgnored private var central: CBCentralManager!
    @ObservationIgnored private var peripheral: CBPeripheral?
    @ObservationIgnored private var captionCharacteristic: CBCharacteristic?
    @ObservationIgnored private var lastSeq: UInt8?
    @ObservationIgnored private var lastPacketSamples = 0
    @ObservationIgnored private var packetCount = 0
    @ObservationIgnored private var rateTask: Task<Void, Never>?
    @ObservationIgnored private var pendingCaption: Data?
    @ObservationIgnored private var latestCaption: (text: String, isFinal: Bool) = ("", true)
    @ObservationIgnored private var lastCaptionSent = Date.distantPast
    @ObservationIgnored private var captionFlushScheduled = false

    override init() {
        super.init()
        // nil queue = main queue, which is what makes the @preconcurrency conformances safe.
        central = CBCentralManager(delegate: self, queue: nil)
        rateTask = Task { [weak self] in
            while !Task.isCancelled {
                try? await Task.sleep(for: .seconds(1))
                guard let self else { return }
                self.packetsPerSecond = self.packetCount
                self.packetCount = 0
            }
        }
    }

    /// Sends the caption to show. Calls are coalesced, so only the newest text is sent.
    /// The newest text is kept while disconnected and sent again after reconnecting.
    func sendCaption(_ text: String, isFinal: Bool) {
        latestCaption = (text, isFinal)
        guard let peripheral, captionCharacteristic != nil else { return }
        let limit = min(Self.captionMaxBytes, peripheral.maximumWriteValueLength(for: .withoutResponse) - 1)
        var payload = Data([isFinal ? 1 : 0])
        payload.append(Self.tail(of: text, maxBytes: limit))
        pendingCaption = payload
        flushCaption()
    }

    private func flushCaption() {
        guard let peripheral, let captionCharacteristic, let pending = pendingCaption else { return }

        let wait = Self.captionMinInterval - Date().timeIntervalSince(lastCaptionSent)
        if wait > 0 {
            guard !captionFlushScheduled else { return }
            captionFlushScheduled = true
            Task { [weak self] in
                try? await Task.sleep(for: .seconds(wait))
                self?.captionFlushScheduled = false
                self?.flushCaption()
            }
            return
        }
        // When the radio is busy, peripheralIsReady(toSendWriteWithoutResponse:) retries.
        guard peripheral.canSendWriteWithoutResponse else { return }

        peripheral.writeValue(pending, for: captionCharacteristic, type: .withoutResponse)
        pendingCaption = nil
        lastCaptionSent = Date()
    }

    /// The end of `text` that fits in `maxBytes` of UTF-8, starting on a word boundary.
    static func tail(of text: String, maxBytes: Int) -> Data {
        var bytes = text.utf8.count
        guard bytes > maxBytes else { return Data(text.utf8) }

        var start = text.startIndex
        while bytes > maxBytes, start < text.endIndex {
            bytes -= text[start].utf8.count
            start = text.index(after: start)
        }
        var tail = text[start...]
        if start > text.startIndex, text[text.index(before: start)] != " ",
           let space = tail.firstIndex(of: " ") {
            tail = tail[tail.index(after: space)...]
        }
        return Data(tail.utf8)
    }

    private func startSearching() {
        if let known = central.retrieveConnectedPeripherals(withServices: [Self.serviceUUID]).first {
            connect(known)
            return
        }
        state = .searching
        central.scanForPeripherals(withServices: [Self.serviceUUID])
    }

    private func connect(_ target: CBPeripheral) {
        peripheral = target
        target.delegate = self
        state = .connecting
        central.connect(target)
    }

    private func resetLink() {
        captionCharacteristic = nil
        pendingCaption = nil
        lastSeq = nil
        mtu = 0
    }

    /// Drops a connection that does not look like the glasses; didDisconnect then
    /// reconnects and discovery runs again from scratch.
    private func abandonConnection(_ peripheral: CBPeripheral, reason: String) {
        print("Glasses link: \(reason); reconnecting")
        central.cancelPeripheralConnection(peripheral)
    }

    private func handleAudioPacket(_ data: Data) {
        let bytes = [UInt8](data)
        guard bytes.count > Self.headerBytes else { return }

        let seq = bytes[0]
        if let lastSeq {
            let gap = Int(seq &- lastSeq &- 1)
            if gap > 0 {
                lostPackets += gap
                if gap <= Self.maxLostPacketsToPad, lastPacketSamples > 0 {
                    onAudio?([Int16](repeating: 0, count: gap * lastPacketSamples))
                }
            }
        }
        lastSeq = seq

        let predictor = Int16(bitPattern: UInt16(bytes[2]) | UInt16(bytes[3]) << 8)
        let samples = ImaAdpcm.decode(bytes[Self.headerBytes...], predictor: predictor, stepIndex: bytes[1])
        lastPacketSamples = samples.count
        packetCount += 1
        onAudio?(samples)
    }
}

extension GlassesLink: @preconcurrency CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            startSearching()
        case .unauthorized:
            state = .unauthorized
        default:
            state = .bluetoothOff
            peripheral = nil
            resetLink()
        }
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        central.stopScan()
        connect(peripheral)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        resetLink()
        mtu = peripheral.maximumWriteValueLength(for: .withoutResponse) + 3
        peripheral.discoverServices([Self.serviceUUID])
    }

    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        resetLink()
        self.peripheral = nil
        startSearching()
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        resetLink()
        // A pending connect never times out, so this reconnects as soon as the glasses
        // are back in range or powered on again.
        state = .connecting
        central.connect(peripheral)
    }
}

extension GlassesLink: @preconcurrency CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard error == nil,
              let service = peripheral.services?.first(where: { $0.uuid == Self.serviceUUID })
        else {
            abandonConnection(peripheral, reason: "service discovery failed: \(String(describing: error))")
            return
        }
        peripheral.discoverCharacteristics([Self.audioUUID, Self.captionUUID], for: service)
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        let characteristics = service.characteristics ?? []
        guard error == nil,
              let audio = characteristics.first(where: { $0.uuid == Self.audioUUID }),
              let caption = characteristics.first(where: { $0.uuid == Self.captionUUID })
        else {
            abandonConnection(peripheral, reason: "characteristics missing: \(String(describing: error))")
            return
        }

        captionCharacteristic = caption
        peripheral.setNotifyValue(true, for: audio)
        state = .connected
        sendCaption(latestCaption.text, isFinal: latestCaption.isFinal)
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateNotificationStateFor characteristic: CBCharacteristic,
                    error: Error?) {
        guard characteristic.uuid == Self.audioUUID else { return }
        if let error {
            abandonConnection(peripheral, reason: "audio subscribe failed: \(error)")
        } else if characteristic.isNotifying {
            state = .streaming
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        guard characteristic.uuid == Self.audioUUID, let data = characteristic.value else { return }
        handleAudioPacket(data)
    }

    func peripheralIsReady(toSendWriteWithoutResponse peripheral: CBPeripheral) {
        flushCaption()
    }
}
