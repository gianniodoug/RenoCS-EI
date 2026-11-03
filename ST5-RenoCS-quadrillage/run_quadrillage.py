"""RenoCS deployment entry point; safe to import in the spawned serial worker."""
if __name__ == "__main__":
    from physical_robot.main import main
    raise SystemExit(main())
