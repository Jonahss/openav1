# Demo streams

`wyldcard_devkit.ivf`: one 640x360 8-bit 4:2:0 key frame of a photograph of the Wyldcard dev kit (Jonah's
e-paper card platform, wyldcard.io), encoded with
`aomenc --ivf --limit=1 --cpu-used=3 --end-usage=q --cq-level=18 --enable-restoration=1 --enable-cdef=1 --kf-max-dist=0`
so that deblocking, CDEF and loop restoration are all exercised. `refout/wyldcard_devkit.yuv` is dav1d's
decoded picture. `tools/demo.sh` decodes it with the RTL and shows both pictures side by side.
