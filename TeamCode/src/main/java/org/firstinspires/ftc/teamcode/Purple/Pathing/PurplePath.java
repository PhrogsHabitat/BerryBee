package org.firstinspires.ftc.teamcode.Purple.Pathing;

import com.pedropathing.paths.Path;

public class PurplePath
{
	private final String name;
	private final Path path;
	private final double durationSec;
	private final double waitTimeSec;
	private Runnable onComplete;
	private Runnable onStart;

	public PurplePath (String name, Path path, double durationSec, double waitTimeSec)
	{
		this.name = name;
		this.path = path;
		this.durationSec = durationSec;
		this.waitTimeSec = waitTimeSec;
	}

	public Path getPath () { return path; }

	public String getName () { return name; }

	public double getDurationSec () { return durationSec; }

	public double getWaitTimeSec () { return waitTimeSec; }

	/**
	 * Set the per-path onStart callback. Runs right when the path begins.
	 *
	 * @param onStart runnable to run when this path starts
	 * @return this (for chaining)
	 */
	public PurplePath onStart (Runnable onStart)
	{
		this.onStart = onStart;
		return this;
	}

	/**
	 * Set the per-path onComplete callback.
	 *
	 * @param onComplete runnable to run when this path completes (or times out)
	 * @return this (for chaining)
	 */
	public PurplePath onComplete (Runnable onComplete)
	{
		this.onComplete = onComplete;
		return this;
	}

	/** Internal: run the configured onStart if present. */
	public void runOnStart ()
	{
		if (onStart != null)
		{
			try
			{
				onStart.run();
			} catch (Exception e)
			{
				e.printStackTrace();
			}
		}
	}

	/** Internal: run the configured onComplete if present. */
	public void runOnComplete ()
	{
		if (onComplete != null)
		{
			try
			{
				onComplete.run();
			} catch (Exception e)
			{
				e.printStackTrace();
			}
		}
	}
}