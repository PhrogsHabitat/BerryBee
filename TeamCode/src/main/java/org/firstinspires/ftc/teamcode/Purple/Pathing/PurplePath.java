package org.firstinspires.ftc.teamcode.Purple.Pathing;

import com.pedropathing.paths.Path; // CHANGED: PathChain no longer exists

/**
 * PurplePath is a small wrapper around a Pedro Path with:
 * - a human-readable name
 * - expected duration (seconds) used as a safety timeout
 * - waitTime (seconds) to wait AFTER this path completes before starting the next path
 * - an optional onComplete runnable that will be executed once the path and its
 *   waitTime have both finished (or the path has timed out)
 */
public class PurplePath
{
	private final String name;
	private final Path path; // CHANGED: was PathChain
	private final double durationSec;
	private final double waitTimeSec;
	private Runnable onComplete;

	/**
	 * Construct a PurplePath.
	 *
	 * @param name        readable name for debugging
	 * @param path        the Pedro Path to follow
	 * @param durationSec expected duration (seconds) — used as a safety timeout
	 * @param waitTimeSec seconds to wait after onComplete before the chain proceeds
	 */
	public PurplePath (String name, Path path, double durationSec, double waitTimeSec)
	{
		this.name = name;
		this.path = path;
		this.durationSec = durationSec;
		this.waitTimeSec = waitTimeSec;
	}

	/** Returns the wrapped Path. */
	public Path getPath () // CHANGED return type
	{
		return path;
	}

	/** Returns the human-readable name. */
	public String getName ()
	{
		return name;
	}

	/** Returns expected duration (seconds). */
	public double getDurationSec ()
	{
		return durationSec;
	}

	/** Returns wait time after completion (seconds). */
	public double getWaitTimeSec ()
	{
		return waitTimeSec;
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