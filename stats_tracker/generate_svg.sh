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
echo "<svg width=\"300px\" height=\"300px\" xmlns=\"http://www.w3.org/2000/svg\">
    <style>
        @import url(\"https://fonts.googleapis.com/css?family=Source+Code+Pro\");
    </style>
    <image href=\"background.png\" width=\"100%\"/>
    <text x=\"50%\" y=\"90px\" font-size=\"65\" text-anchor=\"middle\" fill=\"black\" font-family=\"Source Code Pro\" font-weight=\"bold\">$value</text>
</svg>" > build/loc.svg
cp background.png build/background.png