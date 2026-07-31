namespace MeshOrient.App

open System
open Avalonia
open Avalonia.Controls.ApplicationLifetimes
open Avalonia.Markup.Xaml
open Avalonia.Themes.Fluent

type App() =
    inherit Application()

    override this.Initialize() =
        this.Styles.Add(FluentTheme())
        this.RequestedThemeVariant <- Styling.ThemeVariant.Dark

    override this.OnFrameworkInitializationCompleted() =
        match this.ApplicationLifetime with
        | :? IClassicDesktopStyleApplicationLifetime as desktop ->
            let w = MainWindow()
            desktop.MainWindow <- w
            // A path on the command line loads straight away: this tool gets
            // opened on one scan and closed again, so making that a single
            // step is most of its "lightweight".
            match desktop.Args with
            | null -> ()
            | args when args.Length > 0 -> w.LoadInitial args[0]
            | _ -> ()
        | _ -> ()
        base.OnFrameworkInitializationCompleted()

module Program =

    [<CompiledName "BuildAvaloniaApp">]
    let buildAvaloniaApp () =
        AppBuilder
            .Configure<App>()
            .UsePlatformDetect()
            .WithInterFont()
            .LogToTrace()

    [<EntryPoint; STAThread>]
    let main argv =
        buildAvaloniaApp().StartWithClassicDesktopLifetime argv
