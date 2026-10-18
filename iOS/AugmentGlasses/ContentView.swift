import SwiftUI

struct ContentView: View {
    let session: CaptionSession

    var body: some View {
        NavigationStack {
            VStack(alignment: .leading, spacing: 16) {
                statusPanel

                ScrollViewReader { proxy in
                    ScrollView {
                        Text(session.displayText.isEmpty ? "Captions will appear here." : session.displayText)
                            .font(.title3)
                            .foregroundStyle(session.displayText.isEmpty ? .secondary : .primary)
                            .frame(maxWidth: .infinity, alignment: .leading)
                            .padding()
                            .id("bottom")
                    }
                    .background(Color.gray.opacity(0.1), in: RoundedRectangle(cornerRadius: 12))
                    .onChange(of: session.displayText) {
                        proxy.scrollTo("bottom", anchor: .bottom)
                    }
                }
            }
            .padding()
            .navigationTitle("Augment Glasses")
            .toolbar {
                Button("Clear") { session.clear() }
                    .disabled(session.displayText.isEmpty)
            }
        }
        // Keeps the app in the foreground while testing; see the README about background use.
        .onAppear { UIApplication.shared.isIdleTimerDisabled = true }
        .onDisappear { UIApplication.shared.isIdleTimerDisabled = false }
    }

    private var statusPanel: some View {
        VStack(alignment: .leading, spacing: 6) {
            Label(session.link.state.rawValue, systemImage: linkIcon)
                .font(.headline)
            if session.link.state == .streaming {
                Text("\(session.link.packetsPerSecond) packets/s · \(session.link.lostPackets) lost · MTU \(session.link.mtu)")
                    .font(.caption.monospacedDigit())
                    .foregroundStyle(.secondary)
            }
            Label(session.transcriberStatus, systemImage: "waveform")
                .font(.subheadline)
                .foregroundStyle(.secondary)
        }
    }

    private var linkIcon: String {
        switch session.link.state {
        case .streaming: "dot.radiowaves.left.and.right"
        case .connected, .connecting: "antenna.radiowaves.left.and.right"
        case .searching: "magnifyingglass"
        case .bluetoothOff, .unauthorized: "exclamationmark.triangle"
        }
    }
}
