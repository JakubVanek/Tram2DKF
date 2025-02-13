using Tram2DKF
using Documenter

DocMeta.setdocmeta!(Tram2DKF, :DocTestSetup, :(using Tram2DKF); recursive=true)

makedocs(;
    modules=[Tram2DKF],
    authors="Jakub Vaněk <vanekj19@fel.cvut.cz> and contributors",
    sitename="Tram2DKF",
    format=Documenter.HTML(;
        canonical="https://JakubVanek.github.io/Tram2DKF",
        edit_link="main",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
        "Reference" => "reference.md",
    ],
)

deploydocs(;
    repo="github.com/JakubVanek/Tram2DKF",
    devbranch="main",
)
