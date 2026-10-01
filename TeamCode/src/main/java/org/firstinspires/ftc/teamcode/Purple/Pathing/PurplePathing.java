package org.firstinspires.ftc.teamcode.Purple.Pathing;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PurplePathing
{
	private final Follower follower;

	private final ElapsedTime pathTimer = new ElapsedTime();
	private final ElapsedTime waitTimer = new ElapsedTime();

	// single path control
	private PurplePath currentPath = null;
	private boolean pathActive = false;
	private boolean waitingAfterPath = false;
	private boolean currentHoldEnd = true;

	// chain control
	private PurpleChain currentChain = null;
	private int currentChainIndex = 0;
	private Runnable chainOnComplete = null;
	private boolean chainActive = false;

	/**
	 * Construct the manager with a Pedro Follower instance.
	 *
	 * @param follower Pedro Follower (created using your Constants.createFollower)
	 */
	public PurplePathing (Follower follower)
	{
		this.follower = follower;
		this.pathTimer.reset();
		this.waitTimer.reset();
	}

	/**
	 * Start following a single PurplePath.
	 * The provided path's onComplete callback will be invoked **after the path
	 * completes (or times out) and its configured waitTime has elapsed**.
	 *
	 * @param path    the PurplePath to follow
	 * @param holdEnd whether to hold the drivetrain at the end of the path
	 */
	public void followPath (PurplePath path, boolean holdEnd)
	{
		// cancel any running chain
		currentChain = null;
		chainActive = false;
		chainOnComplete = null;

		startPathInternal(path, holdEnd);
	}

	/**
	 * Start a PurpleChain. The chain's paths are executed in order. After each path finishes
	 * the manager waits the configured waitTime before invoking that path's onComplete and
	 * then starting the next path. When the chain finishes, chainOnComplete runs.
	 *
	 * @param chain           the PurpleChain to run
	 * @param holdEnd         whether each path should be started with holdEnd
	 * @param chainOnComplete optional runnable to run when the entire chain completes
	 */
	public void startChain (PurpleChain chain, boolean holdEnd, Runnable chainOnComplete)
	{
		if (chain == null || chain.getPaths().isEmpty()) return;

		this.currentChain = chain;
		this.currentChainIndex = 0;
		this.chainOnComplete = chainOnComplete;
		this.chainActive = true;
		this.currentHoldEnd = holdEnd;

		// start first
		startPathInternal(chain.getPaths().get(0), holdEnd);
	}

	/**
	 * Must be called inside the OpMode loop() on every iteration.
	 * This updates the follower and handles completion, timeouts, wait timers and chain advancement.
	 */
	public void update ()
	{
		// keep Pedro updated (important)
		follower.update();

		if (pathActive && currentPath != null)
		{
			double elapsed = pathTimer.seconds(); // CHANGED: was getElapsedTimeSeconds()
			if (elapsed >= currentPath.getDurationSec())
			{
				pathActive = false;
				waitingAfterPath = true;
				waitTimer.reset(); // CHANGED: was resetTimer()
			}
		} else if (waitingAfterPath && currentPath != null)
		{
			double waited = waitTimer.seconds(); // CHANGED: was getElapsedTimeSeconds()
			double waitFor = currentPath.getWaitTimeSec();

			if (waited >= waitFor)
			{
				// the post-path delay has finished; now run the per-path callback.
				currentPath.runOnComplete();

				// proceed based on whether we're in a chain or a standalone path
				if (chainActive && currentChain != null)
				{
					// move to the next path in the active chain
					waitingAfterPath = false;
					currentChainIndex++;

					if (currentChainIndex >= currentChain.getPaths().size())
					{
						// entire chain finished
						chainActive = false;
						PurpleChain finishedChain = currentChain;
						currentChain = null;
						currentPath = null;
						if (chainOnComplete != null)
						{
							try
							{
								chainOnComplete.run();
							} catch (Exception e)
							{
								e.printStackTrace();
							}
						}
						finishedChain.runOnComplete();
					} else
					{
						// start the next path in the chain
						PurplePath next = currentChain.getPaths().get(currentChainIndex);
						startPathInternal(next, currentHoldEnd);
					}
				} else
				{
					// single path finished; clear currentPath
					currentPath = null;
					waitingAfterPath = false;
				}
			}
		}
	}

	private void startPathInternal (PurplePath path, boolean holdEnd)
	{
		this.currentPath = path;
		this.pathActive = true;
		this.waitingAfterPath = false;
		this.pathTimer.reset();

		follower.follow(path.getPath());
	}

	/**
	 * Return a human-friendly description of the current path (or NONE).
	 * Includes chain index when a chain is active.
	 */
	public String curPath ()
	{
		if (currentChain != null && chainActive)
		{
			String name = (currentPath == null) ? "NONE" : currentPath.getName();
			return String.format("CHAIN[%d/%d] %s",
					Math.min(currentChainIndex + 1, currentChain.getPaths().size()),
					currentChain.getPaths().size(),
					name);
		} else if (currentPath != null)
		{
			return currentPath.getName();
		} else
		{
			return "NONE";
		}
	}
}