# E

## Extract audio from video

### `MP3`

```bash
ffmpeg -i INPUT_VIDEO -q:a 0 -map a OUTPUT_AUDIO.mp3
```

### lossless `FLAC`

```bash
ffmpeg -i INPUT_VIDEO -c:a flac OUTPUT_AUDIO.flac
```

### uncompressed `WAV`

```bash
ffmpeg -i INPUT_VIDEO OUTPUT_AUDIO.wav
```
