package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

import android.content.Context;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Map;
import java.util.Properties;

/**
 * Simple persistent key-value storage backed by a file on the Robot Controller.
 *
 * Writes are performed to a temporary file and then atomically renamed into
 * place so a crash or power loss during a write cannot corrupt the file.
 */
public class Persist
{
	private static final String FILE_NAME = "PurplePersist.properties";

	private final File file;

	/**
	 * Creates persistent storage using the current Robot Controller app context.
	 */
	public Persist (HardwareMap hardwareMap)
	{
		this(hardwareMap.appContext);
	}

	/**
	 * Creates persistent storage using the provided Android context.
	 */
	public Persist (Context context)
	{
		this.file = new File(context.getFilesDir(), FILE_NAME);
	}

	/**
	 * Saves a key/value pair. If value is null, the key is removed.
	 */
	public synchronized void set (String key, String value)
	{
		validateKey(key);

		Properties properties = loadProperties();
		if (value == null)
		{
			properties.remove(key);
		}
		else
		{
			properties.setProperty(key, value);
		}

		saveProperties(properties);
	}

	/**
	 * Saves multiple key/value pairs in a single load/save cycle.
	 * Null values remove the corresponding key. This is what you want when
	 * several related values (e.g. PrevX/PrevY/PrevHeading) must stay in sync.
	 */
	public synchronized void setAll (Map<String, String> values)
	{
		if (values == null || values.isEmpty())
		{
			return;
		}

		Properties properties = loadProperties();
		for (Map.Entry<String, String> entry : values.entrySet())
		{
			validateKey(entry.getKey());
			if (entry.getValue() == null)
			{
				properties.remove(entry.getKey());
			}
			else
			{
				properties.setProperty(entry.getKey(), entry.getValue());
			}
		}

		saveProperties(properties);
	}

	/**
	 * Gets a value by key, or null when the key does not exist.
	 */
	public synchronized String get (String key)
	{
		validateKey(key);
		return loadProperties().getProperty(key);
	}

	/**
	 * Gets a value by key, or the provided fallback when missing.
	 */
	public synchronized String get (String key, String fallback)
	{
		validateKey(key);
		return loadProperties().getProperty(key, fallback);
	}

	private Properties loadProperties ()
	{
		Properties properties = new Properties();

		if (!file.exists())
		{
			return properties;
		}

		try (FileInputStream input = new FileInputStream(file))
		{
			properties.load(input);
		}
		catch (IOException ignored)
		{
			// Return an empty map on read errors to keep usage simple in OpModes.
		}

		return properties;
	}

	private void saveProperties (Properties properties)
	{
		// Write to a sibling temp file first so a crash during writing can
		// never leave the real file half-written.
		File temp = new File(file.getParentFile(), file.getName() + ".tmp");

		try (FileOutputStream output = new FileOutputStream(temp))
		{
			properties.store(output, "Purple Persist");
			output.getFD().sync();
		}
		catch (IOException e)
		{
			throw new RuntimeException("Failed to write persist temp file", e);
		}

		// renameTo is atomic on the same filesystem (Linux/Android). If it
		// fails because the target already exists, fall back to delete+rename.
		if (!temp.renameTo(file))
		{
			if (file.exists() && !file.delete())
			{
				throw new RuntimeException("Failed to delete old persist file");
			}
			if (!temp.renameTo(file))
			{
				throw new RuntimeException("Failed to replace persist storage file");
			}
		}
	}

	private void validateKey (String key)
	{
		if (key == null || key.trim().isEmpty())
		{
			throw new IllegalArgumentException("Key cannot be null or blank");
		}
	}
}