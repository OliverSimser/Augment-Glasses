import SwiftUI

@main
struct AugmentGlassesApp: App {
    @State private var session = CaptionSession()

    var body: some Scene {
        WindowGroup {
            ContentView(session: session)
                .task { await session.start() }
        }
    }
}
