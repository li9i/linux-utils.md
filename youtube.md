# D

## Download video

[Download yt-dlp_linux](https://github.com/yt-dlp/yt-dlp/releases), e.g. [2025.1.22/yt-dlp_linux](https://github.com/yt-dlp/yt-dlp/releases/download/2025.10.22/yt-dlp_linux). Then

```bash
./yt-dlp_linux  https://www.youtube.com/watch?v=2XcJM6SR2T8&list=PLNqUrPN0FE4Z3bt37kY0mhTLomK0nkxK5
```

## Download video but extract only audio

```bash
./yt-dlp_linux  -x --audio-format mp3  -f bestvideo+bestaudio  -f bestaudio    https://www.youtube.com/playlist?list=FLuPecOb2Lir66VCmnN2eDjQ
```
