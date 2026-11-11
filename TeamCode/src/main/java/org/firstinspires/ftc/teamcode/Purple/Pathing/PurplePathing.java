// PurplePathing.java
package org.firstinspires.ftc.teamcode.Purple.Pathing;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PurplePathing
{
	private final Follower follower;

	// Only need a wait timer now — no more duration/path timer.
	private final ElapsedTime waitTimer = new ElapsedTime();

	// single-path control
	private PurplePath currentPath = null;
	private boolean pathActive = false;      // actively following the path
	private boolean waitingAfterPath = false; // path done, waiting waitTimeSec
	private boolean currentHoldEnd = true;

	// chain control
	private PurpleChain currentChain = null;
	private int currentChainIndex = 0;
	private Runnable chainOnComplete = null;
	private boolean chainActive = false;

	public PurplePathing (Follower follower)
	{
		this.follower = follower;
		this.waitTimer.reset();
	}

	/**
	 * Start following a single PurplePath.
	 * onComplete fires as soon as the follower actually finishes the path.
	 * After that, we wait waitTimeSec before considering the path "over".
	 */
	public void followPath (PurplePath path, boolean holdEnd)
	{
		// cancel any running chain
		currentChain = null;
		chainActive = false;
		chainOnComplete = null;
		currentChainIndex = 0;

		startPathInternal(path, holdEnd);
	}

	public void startChain (PurpleChain chain, boolean holdEnd, Runnable chainOnComplete)
	{
		if (chain == null || chain.getPaths().isEmpty()) return;

		this.currentChain = chain;
		this.currentChainIndex = 0;
		this.chainOnComplete = chainOnComplete;
		this.chainActive = true;
		this.currentHoldEnd = holdEnd;

		startPathInternal(chain.getPaths().get(0), holdEnd);
	}

	public void update ()
	{
		// keep Pedro updated
		follower.update();

		if (currentPath == null) return;

		if (pathActive)
		{
			// Ask Pedro whether the path is done.
			//  - !isBusy():        path finished without holdEnd
			//  - atParametricEnd(): path hit its endpoint (works with holdEnd=true)
			if (!follower.isBusy() || follower.atParametricEnd())
			{
				pathActive = false;
				waitingAfterPath = true;
				waitTimer.reset();

				// Fire onComplete exactly once, right when the path finishes.
				currentPath.runOnComplete();
			}
		}
		else if (waitingAfterPath)
		{
			// Wait waitTimeSec, then advance.
			if (waitTimer.seconds() >= currentPath.getWaitTimeSec())
			{
				waitingAfterPath = false;

				if (chainActive && currentChain != null)
				{
					currentChainIndex++;

					if (currentChainIndex >= currentChain.getPaths().size())
					{
						// Entire chain finished.
						chainActive = false;
						PurpleChain finishedChain = currentChain;
						currentChain = null;
						currentPath = null;

						if (chainOnComplete != null)
						{
							try { chainOnComplete.run(); }
							catch (Exception e) { e.printStackTrace(); }
						}
						finishedChain.runOnComplete();
					}
					else
					{
						PurplePath next = currentChain.getPaths().get(currentChainIndex);
						startPathInternal(next, currentHoldEnd);
					}
				}
				else
				{
					// Standalone path finished.
					currentPath = null;
				}
			}
		}
	}

	private void startPathInternal (PurplePath path, boolean holdEnd)
	{
		this.currentPath = path;
		this.pathActive = true;
		this.waitingAfterPath = false;

		// onStart fires right when we hand the path to Pedro.
		path.runOnStart();

		follower.holdEnd.set(true);
		follower.follow(path.getPath());
	}

	public String curPath ()
	{
		if (currentChain != null && chainActive)
		{
			String name = (currentPath == null) ? "NONE" : currentPath.getName();
			return String.format("CHAIN[%d/%d] %s",
					Math.min(currentChainIndex + 1, currentChain.getPaths().size()),
					currentChain.getPaths().size(),
					name);
		}
		else if (currentPath != null)
		{
			return currentPath.getName();
		}
		else
		{
			return "NONE";
		}
	}
}