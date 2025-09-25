# NES_DE2_70.sdc - Fichier de contraintes basique

# Déclaration de l'horloge principale (remplacez clk par le bon nom de port)
create_clock -name iCLK_50 -period 20.0 [get_ports {iCLK_50}]

# D'autres contraintes peuvent être ajoutées ici
create_generated_clock \
  -name clk_21mhz \
  -source [get_pins {clk_21mhz_inst|altpll_component|pll|inclk[0]}] \
  -multiply_by 3 \
  -divide_by 7 \
  [get_pins {clk_21mhz_inst|altpll_component|pll|clk[0]}]