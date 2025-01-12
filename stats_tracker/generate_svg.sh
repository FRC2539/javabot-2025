read f i d <<< $(
    git log --shortstat --all | \
    awk '/^ [0-9]/ { f += $1; i += $4; d += $6 } \
    END { printf("%d %d %d", f, i, d) }'
)
echo "File(s) changed: $f"
echo "Insertion(s): $i"
echo "Deletion(s): $d"
value=$(($i - $d))
echo "Lines of code: $value"
mkdir build
python html-to-img.py $value
