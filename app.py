"""Hoberman sphere designer as a native desktop app -- no server.

Launch by double-clicking "Hoberman Designer.app" (or `open "Hoberman
Designer.app"`).  The app talks to Python directly through pywebview's JS
bridge; nothing listens on any port.

IMPORTANT (macOS): the window only appears when launched as the .app bundle.
A bare `python app.py` from a terminal inherits a background activation
context, so the process runs but no window is shown -- LaunchServices needs
the bundle's Info.plist to register it as a foreground GUI app.  Set
HOBERMAN_DIAG=1 to trace startup to /tmp/hoberman_status.log.
"""

import base64
import os
import sys

import webview

import designer as D

HERE = os.path.dirname(os.path.abspath(__file__))
DIAG = os.environ.get('HOBERMAN_DIAG')


def _diag(msg):
    if DIAG:
        with open('/tmp/hoberman_status.log', 'a') as f:
            f.write(msg + '\n')


class Api:
    def build(self, params):
        r = D.do_build(params)
        if r.get('ok'):
            r['stl_print'] = base64.b64encode(r.pop('stl_print')).decode()
            r['stl_open'] = base64.b64encode(r.pop('stl_open')).decode()
        return r

    def check(self):
        return D.start_check()

    def check_status(self):
        return D.check_status()

    def export_stl(self):
        return D.start_export()

    def export_status(self):
        return D.export_status()


def _bring_to_front():
    """Force the process to a regular foreground app and raise the window.
    Harmless if the bundle already did so; the safety net for odd launches."""
    try:
        import AppKit
        AppKit.NSApplication.sharedApplication().setActivationPolicy_(0)
        AppKit.NSRunningApplication.currentApplication().activateWithOptions_(
            AppKit.NSApplicationActivateIgnoringOtherApps)
    except Exception as e:  # noqa: BLE001
        _diag(f'_bring_to_front error: {e}')


def main():
    _diag(f'main() start cwd={os.getcwd()} exe={sys.executable}')
    window = webview.create_window('Hoberman Sphere Designer',
                                   html=D.load_ui(app_mode=True),
                                   js_api=Api(), width=1560, height=980)
    try:
        window.events.loaded += _bring_to_front
        window.events.shown += lambda: _diag('event: shown')
        window.events.loaded += lambda: _diag('event: loaded')
    except Exception as e:  # noqa: BLE001
        _diag(f'event hook error: {e}')

    def selftest(win):
        import time
        for _ in range(60):
            time.sleep(1)
            try:
                gen = win.evaluate_js('window._gen')
            except Exception:
                gen = None
            if gen:
                tris = win.evaluate_js(
                    'document.querySelectorAll("#derived tr").length')
                print(f'SELFTEST PASS: first build gen={gen}, '
                      f'derived rows={tris}', flush=True)
                win.destroy()
                return
        print('SELFTEST FAIL: page never completed a build', flush=True)
        win.destroy()

    if '--selftest' in sys.argv:
        webview.start(selftest, window)
    else:
        webview.start()


if __name__ == '__main__':
    main()
