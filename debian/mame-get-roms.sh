    DESTPATH="/usr/share/games/mame/roms/";
    FREEROMS=(
      'Robby Roto (Ballt/Midway)'  'https://www.mamedev.org/roms/robby/robby.zip' \
      'Gridlee (Videa)'            'https://www.mamedev.org/roms/gridlee/gridlee.zip' \
      'Alien Arena (Duncan Brown)' 'https://www.mamedev.org/roms/alienar/alienar.zip' \
      'Teeter Torture (Exidy)'     'https://www.mamedev.org/roms/teetert/teetert.zip' \
      'Circus (Exidy)'             'https://www.mamedev.org/roms/circus/circus.zip' \
      'Robot Bowl (Exidy)'         'https://www.mamedev.org/roms/robotbwl/robotbwl.zip' \
      'Car Polo (Exidy)'           'https://www.mamedev.org/roms/carpolo/carpolo.zip' \
      'Rip Cord (Exidy)'           'https://www.mamedev.org/roms/ripcord/ripcord.zip' \
      'Fire One (Exidy)'           'https://www.mamedev.org/roms/fireone/fireone.zip' \
      'Crash (Exidy)'              'https://www.mamedev.org/roms/crash/crash.zip' \
      'Star Fire (Exidy)'          'https://www.mamedev.org/roms/starfire/starfire.zip' \
      'Side Track (Exidy)'         'https://www.mamedev.org/roms/sidetrac/sidetrac.zip' \
      'Spectar (Exidy)'            'https://www.mamedev.org/roms/spectar/spectar.zip' \
      'Targ (Exidy)'               'https://www.mamedev.org/roms/targ/targ.zip' \
    );
    NBROMS=$((${#FREEROMS[@]}/2));

    if [ ! -d ${DESTPATH} ]; then
      mkdir -p ${DESTPATH}
    fi

    for (( i = 0 ; i < ${#FREEROMS[@]} ; i=i+2 ))
    do
      ROMPATH=${DESTPATH}$(basename ${FREEROMS[$i+1]});
      if [ ! -s $ROMPATH ]; then
        NOTDOWNLOADED="true";
        break;
      fi;
    done

    if [ "$NOTDOWNLOADED" = "true" ]; then
      db_get mame/getfreeroms
      if [ "$RET" = "true" ]; then
        for (( i = 0 ; i < ${#FREEROMS[@]} ; i=i+2 ))
        do
          ROMPATH=${DESTPATH}$(basename ${FREEROMS[$i+1]});
          #echo -en "\n-- $(((i+1)/2))/${NBROMS}. ${FREEROMS[$i]} ";
          wget --user-agent="MAME package for Debian/Ubuntu <ludomatic@gmail.com>" ${FREEROMS[$i+1]} -O "$ROMPATH";
          if [ $? -ne 0 ]; then NOTCOMPLETE="true" ; fi;
          if [ ! -s $ROMPATH ]; then NOTCOMPLETE="true" ; fi;
        done
      fi
    fi
