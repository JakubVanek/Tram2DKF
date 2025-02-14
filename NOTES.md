# Miscellaneous notes for presentation

(Czech)

* můj cíl -> trochu si pohrát s typovým systémem
  * "traits" `StateEquation{ContinuousTime}`, `StateEquation{DiscreteTime}`
  * multiple dispatch:
    * různé typy Kalmanových filtrů
    * specializace pro sqrt filtry, pokud je všechno `SqrtGaussian`
* původně jsem mířil i na výkon, ale to už jsem nestihnul
  * motivací bylo hledat optimální kovarianci procesního šumu
  * co jsem stihnul a co ne:
    * DONE: struktury mají definované typy prvků
    * DONE: generátor trajektorie - barrier funkce
      * pole pro definici segmentů je efektivně `Vector{Any}`
      * multiple dispatch by byl v integrační smyčce pomalý
      * -> rozdělení na dvě smyčky
        * vnější používá multiple dispatch; jedna iterace = jeden přechod mezi segmenty
        * vnitřní funkce by měla být specializovaná pro aktuální typ segmentu
    * TODO: minimálně `render_trip()` hodně alokuje, nápady:
      * StaticArrays
      * vytvořit variantu definice modelů, kde se nevrací `Vector`,
        ale funkce jenom zapíše derivace do výstupního argumentu
