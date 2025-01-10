value=$1
echo "<svg width=\"300px\" height=\"300px\" xmlns=\"http://www.w3.org/2000/svg\">
    <style>
        @import url(\"https://fonts.googleapis.com/css?family=Source+Code+Pro\");
    </style>
    <image href=\"background.png\" width=\"100%\"/>
    <text x=\"50%\" y=\"90px\" font-size=\"65\" text-anchor=\"middle\" fill=\"black\" font-family=\"Source Code Pro\" font-weight=\"bold\">$value</text>
</svg>" > loc.svg