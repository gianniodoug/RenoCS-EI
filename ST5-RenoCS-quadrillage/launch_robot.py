"""Desktop entry bootstrap; importing/spawning NEVER opens a GUI or hardware."""
if __name__ == "__main__":
    import os
    from pathlib import Path
    import sys
    import traceback
    from datetime import datetime
    import subprocess

    folder = Path(__file__).resolve().parent
    sys.path.insert(0, str(folder))
    sys.dont_write_bytecode = True
    os.environ["PYTHONDONTWRITEBYTECODE"] = "1"
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    log = folder / "logs" / ("robot-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".log")
    try:
        log.parent.mkdir(exist_ok=True)
        stream = open(log, "a", buffering=1, encoding="utf-8")
        sys.stdout = sys.stderr = stream
        os.dup2(stream.fileno(), 1)
        os.dup2(stream.fileno(), 2)
        from physical_robot.gui import main
        raise SystemExit(main(folder, log))
    except Exception as error:
        traceback.print_exc()
        message = "RenoCS could not start: " + str(error)[:180] + "\nLogs: " + str(log.parent)
        try:
            import tkinter as tk
            from tkinter import messagebox
            root = tk._default_root or tk.Tk()
            root.withdraw()
            messagebox.showerror("RenoCS launch failed", message, parent=root)
            root.destroy()
            raise SystemExit(2)
        except Exception:
            message += "\nInstall python3-tk using Raspberry Pi OS Add/Remove Software."
        for command in (["zenity", "--error", "--text=" + message], ["xmessage", "-center", message]):
            try:
                subprocess.run(command, timeout=30, check=False)
                break
            except (OSError, subprocess.TimeoutExpired):
                continue
        raise SystemExit(2)
