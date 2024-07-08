import imageio_ffmpeg as ffmpeg
import subprocess
import os
from datetime import datetime

def start_recording(output_dir, input_url, duration=30, fps=25):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = os.path.join(output_dir, f"recording_{timestamp}.mp4")

    ffmpeg_exe = ffmpeg.get_ffmpeg_exe()

    ffmpeg_command = [
        ffmpeg_exe,
        '-i', input_url,
        '-framerate', str(fps),
        '-vsync', 'vfr',
        '-c:v', 'libx264',
        '-t', str(duration),
        '-movflags', '+faststart',
        output_path
    ]

    process = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    print("Recording completed.")
    print("FFmpeg stdout:", stdout.decode())
    print("FFmpeg stderr:", stderr.decode())

if __name__ == "__main__":
    output_directory = "/home/nick/video"
    input_stream_url = "http://192.168.1.100:8090/?action=stream"

    # Запись одного 60-секундного ролика
    start_recording(output_directory, input_stream_url, duration=60, fps=19)




output_directory = "/home/nick/video"