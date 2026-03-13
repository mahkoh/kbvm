use super::*;

#[cfg(test)]
pub(super) const LEN: usize = 2496;

pub(super) const LONGEST_NAME: usize = 27;

pub(super) static NAMES: &str = "NoSymbolspaceexclamquotedblnumbersigndollarpercentampersandapostrophequoterightparenleftparenrightasteriskpluscommaminusperiodslash0123456789colonsemicolonlessequalgreaterquestionatABCDEFGHIJKLMNOPQRSTUVWXYZbracketleftbackslashbracketrightasciicircumunderscoregravequoteleftabcdefghijklmnopqrstuvwxyzbraceleftbarbracerightasciitildenobreakspaceexclamdowncentsterlingcurrencyyenbrokenbarsectiondiaeresiscopyrightordfeminineguillemotleftguillemetleftnotsignhyphenregisteredmacrondegreeplusminustwosuperiorthreesuperioracutemuparagraphperiodcenteredcedillaonesuperiormasculineordmasculineguillemotrightguillemetrightonequarteronehalfthreequartersquestiondownAgraveAacuteAcircumflexAtildeAdiaeresisAringAECcedillaEgraveEacuteEcircumflexEdiaeresisIgraveIacuteIcircumflexIdiaeresisETHEthNtildeOgraveOacuteOcircumflexOtildeOdiaeresismultiplyOslashOobliqueUgraveUacuteUcircumflexUdiaeresisYacuteTHORNThornssharpagraveaacuteacircumflexatildeadiaeresisaringaeccedillaegraveeacuteecircumflexediaeresisigraveiacuteicircumflexidiaeresisethntildeograveoacuteocircumflexotildeodiaeresisdivisionoslashoobliqueugraveuacuteucircumflexudiaeresisyacutethornydiaeresisAogonekbreveLstrokeLcaronSacuteScaronScedillaTcaronZacuteZcaronZabovedotaogonekogoneklstrokelcaronsacutecaronscaronscedillatcaronzacutedoubleacutezcaronzabovedotRacuteAbreveLacuteCacuteCcaronEogonekEcaronDcaronDstrokeNacuteNcaronOdoubleacuteRcaronUringUdoubleacuteTcedillaracuteabrevelacutecacuteccaroneogonekecarondcarondstrokenacutencaronodoubleacutercaronuringudoubleacutetcedillaabovedotHstrokeHcircumflexIabovedotGbreveJcircumflexhstrokehcircumflexidotlessgbrevejcircumflexCabovedotCcircumflexGabovedotGcircumflexUbreveScircumflexcabovedotccircumflexgabovedotgcircumflexubrevescircumflexkrakappaRcedillaItildeLcedillaEmacronGcedillaTslashrcedillaitildelcedillaemacrongcedillatslashENGengAmacronIogonekEabovedotImacronNcedillaOmacronKcedillaUogonekUtildeUmacronamacroniogonekeabovedotimacronncedillaomacronkcedillauogonekutildeumacronoverlinekana_fullstopkana_openingbracketkana_closingbracketkana_commakana_conjunctivekana_middledotkana_WOkana_akana_ikana_ukana_ekana_okana_yakana_yukana_yokana_tsukana_tuprolongedsoundkana_Akana_Ikana_Ukana_Ekana_Okana_KAkana_KIkana_KUkana_KEkana_KOkana_SAkana_SHIkana_SUkana_SEkana_SOkana_TAkana_CHIkana_TIkana_TSUkana_TUkana_TEkana_TOkana_NAkana_NIkana_NUkana_NEkana_NOkana_HAkana_HIkana_FUkana_HUkana_HEkana_HOkana_MAkana_MIkana_MUkana_MEkana_MOkana_YAkana_YUkana_YOkana_RAkana_RIkana_RUkana_REkana_ROkana_WAkana_NvoicedsoundsemivoicedsoundArabic_commaArabic_semicolonArabic_question_markArabic_hamzaArabic_maddaonalefArabic_hamzaonalefArabic_hamzaonwawArabic_hamzaunderalefArabic_hamzaonyehArabic_alefArabic_behArabic_tehmarbutaArabic_tehArabic_thehArabic_jeemArabic_hahArabic_khahArabic_dalArabic_thalArabic_raArabic_zainArabic_seenArabic_sheenArabic_sadArabic_dadArabic_tahArabic_zahArabic_ainArabic_ghainArabic_tatweelArabic_fehArabic_qafArabic_kafArabic_lamArabic_meemArabic_noonArabic_haArabic_hehArabic_wawArabic_alefmaksuraArabic_yehArabic_fathatanArabic_dammatanArabic_kasratanArabic_fathaArabic_dammaArabic_kasraArabic_shaddaArabic_sukunSerbian_djeMacedonia_gjeCyrillic_ioUkrainian_ieUkranian_jeMacedonia_dseUkrainian_iUkranian_iUkrainian_yiUkranian_yiCyrillic_jeSerbian_jeCyrillic_ljeSerbian_ljeCyrillic_njeSerbian_njeSerbian_tsheMacedonia_kjeUkrainian_ghe_with_upturnByelorussian_shortuCyrillic_dzheSerbian_dzenumerosignSerbian_DJEMacedonia_GJECyrillic_IOUkrainian_IEUkranian_JEMacedonia_DSEUkrainian_IUkranian_IUkrainian_YIUkranian_YICyrillic_JESerbian_JECyrillic_LJESerbian_LJECyrillic_NJESerbian_NJESerbian_TSHEMacedonia_KJEUkrainian_GHE_WITH_UPTURNByelorussian_SHORTUCyrillic_DZHESerbian_DZECyrillic_yuCyrillic_aCyrillic_beCyrillic_tseCyrillic_deCyrillic_ieCyrillic_efCyrillic_gheCyrillic_haCyrillic_iCyrillic_shortiCyrillic_kaCyrillic_elCyrillic_emCyrillic_enCyrillic_oCyrillic_peCyrillic_yaCyrillic_erCyrillic_esCyrillic_teCyrillic_uCyrillic_zheCyrillic_veCyrillic_softsignCyrillic_yeruCyrillic_zeCyrillic_shaCyrillic_eCyrillic_shchaCyrillic_cheCyrillic_hardsignCyrillic_YUCyrillic_ACyrillic_BECyrillic_TSECyrillic_DECyrillic_IECyrillic_EFCyrillic_GHECyrillic_HACyrillic_ICyrillic_SHORTICyrillic_KACyrillic_ELCyrillic_EMCyrillic_ENCyrillic_OCyrillic_PECyrillic_YACyrillic_ERCyrillic_ESCyrillic_TECyrillic_UCyrillic_ZHECyrillic_VECyrillic_SOFTSIGNCyrillic_YERUCyrillic_ZECyrillic_SHACyrillic_ECyrillic_SHCHACyrillic_CHECyrillic_HARDSIGNGreek_ALPHAaccentGreek_EPSILONaccentGreek_ETAaccentGreek_IOTAaccentGreek_IOTAdieresisGreek_IOTAdiaeresisGreek_OMICRONaccentGreek_UPSILONaccentGreek_UPSILONdieresisGreek_OMEGAaccentGreek_accentdieresisGreek_horizbarGreek_alphaaccentGreek_epsilonaccentGreek_etaaccentGreek_iotaaccentGreek_iotadieresisGreek_iotaaccentdieresisGreek_omicronaccentGreek_upsilonaccentGreek_upsilondieresisGreek_upsilonaccentdieresisGreek_omegaaccentGreek_ALPHAGreek_BETAGreek_GAMMAGreek_DELTAGreek_EPSILONGreek_ZETAGreek_ETAGreek_THETAGreek_IOTAGreek_KAPPAGreek_LAMDAGreek_LAMBDAGreek_MUGreek_NUGreek_XIGreek_OMICRONGreek_PIGreek_RHOGreek_SIGMAGreek_TAUGreek_UPSILONGreek_PHIGreek_CHIGreek_PSIGreek_OMEGAGreek_alphaGreek_betaGreek_gammaGreek_deltaGreek_epsilonGreek_zetaGreek_etaGreek_thetaGreek_iotaGreek_kappaGreek_lamdaGreek_lambdaGreek_muGreek_nuGreek_xiGreek_omicronGreek_piGreek_rhoGreek_sigmaGreek_finalsmallsigmaGreek_tauGreek_upsilonGreek_phiGreek_chiGreek_psiGreek_omegaleftradicaltopleftradicalhorizconnectortopintegralbotintegralvertconnectortopleftsqbracketbotleftsqbrackettoprightsqbracketbotrightsqbrackettopleftparensbotleftparenstoprightparensbotrightparensleftmiddlecurlybracerightmiddlecurlybracetopleftsummationbotleftsummationtopvertsummationconnectorbotvertsummationconnectortoprightsummationbotrightsummationrightmiddlesummationlessthanequalnotequalgreaterthanequalintegralthereforevariationinfinitynablaapproximatesimilarequalifonlyifimpliesidenticalradicalincludedinincludesintersectionunionlogicalandlogicalorpartialderivativefunctionleftarrowuparrowrightarrowdownarrowblanksoliddiamondcheckerboardhtffcrlfnlvtlowrightcorneruprightcornerupleftcornerlowleftcornercrossinglineshorizlinescan1horizlinescan3horizlinescan5horizlinescan7horizlinescan9lefttrighttbotttoptvertbaremspaceenspaceem3spaceem4spacedigitspacepunctspacethinspacehairspaceemdashendashsignifblankellipsisdoubbaselinedotonethirdtwothirdsonefifthtwofifthsthreefifthsfourfifthsonesixthfivesixthscareoffigdashleftanglebracketdecimalpointrightanglebracketmarkeroneeighththreeeighthsfiveeighthsseveneighthstrademarksignaturemarktrademarkincircleleftopentrianglerightopentriangleemopencircleemopenrectangleleftsinglequotemarkrightsinglequotemarkleftdoublequotemarkrightdoublequotemarkprescriptionpermilleminutessecondslatincrosshexagramfilledrectbulletfilledlefttribulletfilledrighttribulletemfilledcircleemfilledrectenopencircbulletenopensquarebulletopenrectbulletopentribulletupopentribulletdownopenstarenfilledcircbulletenfilledsqbulletfilledtribulletupfilledtribulletdownleftpointerrightpointerclubdiamondheartmaltesecrossdaggerdoubledaggercheckmarkballotcrossmusicalsharpmusicalflatmalesymbolfemalesymboltelephonetelephonerecorderphonographcopyrightcaretsinglelowquotemarkdoublelowquotemarkcursorleftcaretrightcaretdowncaretupcaretoverbardowntackupshoedownstileunderbarjotquaduptackcircleupstiledownshoerightshoeleftshoelefttackrighttackhebrew_doublelowlinehebrew_alephhebrew_bethebrew_bethhebrew_gimelhebrew_gimmelhebrew_dalethebrew_dalethhebrew_hehebrew_wawhebrew_zainhebrew_zayinhebrew_chethebrew_hethebrew_tethebrew_tethhebrew_yodhebrew_finalkaphhebrew_kaphhebrew_lamedhebrew_finalmemhebrew_memhebrew_finalnunhebrew_nunhebrew_samechhebrew_samekhhebrew_ayinhebrew_finalpehebrew_pehebrew_finalzadehebrew_finalzadihebrew_zadehebrew_zadihebrew_qophhebrew_kufhebrew_reshhebrew_shinhebrew_tawhebrew_tafThai_kokaiThai_khokhaiThai_khokhuatThai_khokhwaiThai_khokhonThai_khorakhangThai_ngonguThai_chochanThai_chochingThai_chochangThai_sosoThai_chochoeThai_yoyingThai_dochadaThai_topatakThai_thothanThai_thonangmonthoThai_thophuthaoThai_nonenThai_dodekThai_totaoThai_thothungThai_thothahanThai_thothongThai_nonuThai_bobaimaiThai_poplaThai_phophungThai_fofaThai_phophanThai_fofanThai_phosamphaoThai_momaThai_yoyakThai_roruaThai_ruThai_lolingThai_luThai_wowaenThai_sosalaThai_sorusiThai_sosuaThai_hohipThai_lochulaThai_oangThai_honokhukThai_paiyannoiThai_saraaThai_maihanakatThai_saraaaThai_saraamThai_saraiThai_saraiiThai_saraueThai_saraueeThai_sarauThai_sarauuThai_phinthuThai_maihanakat_maithoThai_bahtThai_saraeThai_saraaeThai_saraoThai_saraaimaimuanThai_saraaimaimalaiThai_lakkhangyaoThai_maiyamokThai_maitaikhuThai_maiekThai_maithoThai_maitriThai_maichattawaThai_thanthakhatThai_nikhahitThai_leksunThai_leknungThai_leksongThai_leksamThai_leksiThai_lekhaThai_lekhokThai_lekchetThai_lekpaetThai_lekkaoHangul_KiyeogHangul_SsangKiyeogHangul_KiyeogSiosHangul_NieunHangul_NieunJieujHangul_NieunHieuhHangul_DikeudHangul_SsangDikeudHangul_RieulHangul_RieulKiyeogHangul_RieulMieumHangul_RieulPieubHangul_RieulSiosHangul_RieulTieutHangul_RieulPhieufHangul_RieulHieuhHangul_MieumHangul_PieubHangul_SsangPieubHangul_PieubSiosHangul_SiosHangul_SsangSiosHangul_IeungHangul_JieujHangul_SsangJieujHangul_CieucHangul_KhieuqHangul_TieutHangul_PhieufHangul_HieuhHangul_AHangul_AEHangul_YAHangul_YAEHangul_EOHangul_EHangul_YEOHangul_YEHangul_OHangul_WAHangul_WAEHangul_OEHangul_YOHangul_UHangul_WEOHangul_WEHangul_WIHangul_YUHangul_EUHangul_YIHangul_IHangul_J_KiyeogHangul_J_SsangKiyeogHangul_J_KiyeogSiosHangul_J_NieunHangul_J_NieunJieujHangul_J_NieunHieuhHangul_J_DikeudHangul_J_RieulHangul_J_RieulKiyeogHangul_J_RieulMieumHangul_J_RieulPieubHangul_J_RieulSiosHangul_J_RieulTieutHangul_J_RieulPhieufHangul_J_RieulHieuhHangul_J_MieumHangul_J_PieubHangul_J_PieubSiosHangul_J_SiosHangul_J_SsangSiosHangul_J_IeungHangul_J_JieujHangul_J_CieucHangul_J_KhieuqHangul_J_TieutHangul_J_PhieufHangul_J_HieuhHangul_RieulYeorinHieuhHangul_SunkyeongeumMieumHangul_SunkyeongeumPieubHangul_PanSiosHangul_KkogjiDalrinIeungHangul_SunkyeongeumPhieufHangul_YeorinHieuhHangul_AraeAHangul_AraeAEHangul_J_PanSiosHangul_J_KkogjiDalrinIeungHangul_J_YeorinHieuhKorean_WonOEoeYdiaeresisEuroSign3270_Duplicate3270_FieldMark3270_Right23270_Left23270_BackTab3270_EraseEOF3270_EraseInput3270_Reset3270_Quit3270_PA13270_PA23270_PA33270_Test3270_Attn3270_CursorBlink3270_AltCursor3270_KeyClick3270_Jump3270_Ident3270_Rule3270_Copy3270_Play3270_Setup3270_Record3270_ChangeScreen3270_DeleteWord3270_ExSelect3270_CursorSelect3270_PrintScreen3270_EnterISO_LockISO_Level2_LatchISO_Level3_ShiftISO_Level3_LatchISO_Level3_LockISO_Group_LatchISO_Group_LockISO_Next_GroupISO_Next_Group_LockISO_Prev_GroupISO_Prev_Group_LockISO_First_GroupISO_First_Group_LockISO_Last_GroupISO_Last_Group_LockISO_Level5_ShiftISO_Level5_LatchISO_Level5_LockISO_Left_TabISO_Move_Line_UpISO_Move_Line_DownISO_Partial_Line_UpISO_Partial_Line_DownISO_Partial_Space_LeftISO_Partial_Space_RightISO_Set_Margin_LeftISO_Set_Margin_RightISO_Release_Margin_LeftISO_Release_Margin_RightISO_Release_Both_MarginsISO_Fast_Cursor_LeftISO_Fast_Cursor_RightISO_Fast_Cursor_UpISO_Fast_Cursor_DownISO_Continuous_UnderlineISO_Discontinuous_UnderlineISO_EmphasizeISO_Center_ObjectISO_Enterdead_gravedead_acutedead_circumflexdead_tildedead_perispomenidead_macrondead_brevedead_abovedotdead_diaeresisdead_aboveringdead_doubleacutedead_carondead_cedilladead_ogonekdead_iotadead_voiced_sounddead_semivoiced_sounddead_belowdotdead_hookdead_horndead_strokedead_abovecommadead_psilidead_abovereversedcommadead_dasiadead_doublegravedead_belowringdead_belowmacrondead_belowcircumflexdead_belowtildedead_belowbrevedead_belowdiaeresisdead_invertedbrevedead_belowcommadead_currencyAccessX_EnableAccessX_Feedback_EnableRepeatKeys_EnableSlowKeys_EnableBounceKeys_EnableStickyKeys_EnableMouseKeys_EnableMouseKeys_Accel_EnableOverlay1_EnableOverlay2_EnableAudibleBell_Enabledead_adead_Adead_edead_Edead_idead_Idead_odead_Odead_udead_Udead_small_schwadead_schwadead_capital_schwadead_SCHWAdead_greekdead_hamzadead_lowlinedead_aboveverticallinedead_belowverticallinedead_longsolidusoverlaychChCHc_hC_hC_HFirst_Virtual_ScreenPrev_Virtual_ScreenNext_Virtual_ScreenLast_Virtual_ScreenTerminate_ServerPointer_LeftPointer_RightPointer_UpPointer_DownPointer_UpLeftPointer_UpRightPointer_DownLeftPointer_DownRightPointer_Button_DfltPointer_Button1Pointer_Button2Pointer_Button3Pointer_Button4Pointer_Button5Pointer_DblClick_DfltPointer_DblClick1Pointer_DblClick2Pointer_DblClick3Pointer_DblClick4Pointer_DblClick5Pointer_Drag_DfltPointer_Drag1Pointer_Drag2Pointer_Drag3Pointer_Drag4Pointer_EnableKeysPointer_AcceleratePointer_DfltBtnNextPointer_DfltBtnPrevPointer_Drag5BackSpaceTabLinefeedClearReturnPauseScroll_LockSys_ReqEscapeMulti_keySunComposeKanjiMuhenkanHenkan_ModeHenkanRomajiHiraganaKatakanaHiragana_KatakanaZenkakuHankakuZenkaku_HankakuTourokuMassyoKana_LockKana_ShiftEisu_ShiftEisu_toggleHangulHangul_StartHangul_EndHangul_HanjaHangul_JamoHangul_RomajaCodeinputKanji_BangouHangul_CodeinputHangul_JeonjaHangul_BanjaHangul_PreHanjaHangul_PostHanjaSingleCandidateHangul_SingleCandidateMultipleCandidateZen_KohoHangul_MultipleCandidatePreviousCandidateMae_KohoHangul_PreviousCandidateHangul_SpecialHomeLeftUpRightDownPriorPage_UpSunPageUpNextPage_DownSunPageDownEndBeginSelectPrintSunPrint_ScreenExecuteInsertUndoSunUndoRedoSunAgainMenuFindSunFindCancelSunStopHelpBreakMode_switchscript_switchISO_Group_Shiftkana_switchArabic_switchGreek_switchHebrew_switchHangul_switchSunAltGraphNum_LockKP_SpaceKP_TabKP_EnterKP_F1KP_F2KP_F3KP_F4KP_HomeKP_LeftKP_UpKP_RightKP_DownKP_PriorKP_Page_UpKP_NextKP_Page_DownKP_EndKP_BeginKP_InsertKP_DeleteKP_MultiplyKP_AddKP_SeparatorKP_SubtractKP_DecimalKP_DivideKP_0KP_1KP_2KP_3KP_4KP_5KP_6KP_7KP_8KP_9KP_EqualF1F2F3F4F5F6F7F8F9F10F11L1F12L2F13L3F14L4F15L5F16L6F17L7F18L8F19L9F20L10F21R1F22R2F23R3F24R4F25R5F26R6F27R7F28R8F29R9F30R10F31R11F32R12F33R13F34R14F35R15Shift_LShift_RControl_LControl_RCaps_LockShift_LockMeta_LMeta_RAlt_LAlt_RSuper_LSuper_RHyper_LHyper_Rbraille_dot_1braille_dot_2braille_dot_3braille_dot_4braille_dot_5braille_dot_6braille_dot_7braille_dot_8braille_dot_9braille_dot_10DeleteVoidSymbolIbreveibreveWcircumflexwcircumflexYcircumflexycircumflexSCHWAObarredOhornohornUhornuhornZstrokezstrokeEZHOcaronocaronGcarongcaronschwaobarredezhcombining_gravecombining_acutecombining_tildecombining_hookcombining_belowdotCyrillic_GHE_barCyrillic_ghe_barCyrillic_ZHE_descenderCyrillic_zhe_descenderCyrillic_KA_descenderCyrillic_ka_descenderCyrillic_KA_vertstrokeCyrillic_ka_vertstrokeCyrillic_EN_descenderCyrillic_en_descenderCyrillic_U_straightCyrillic_u_straightCyrillic_U_straight_barCyrillic_u_straight_barCyrillic_HA_descenderCyrillic_ha_descenderCyrillic_CHE_descenderCyrillic_che_descenderCyrillic_CHE_vertstrokeCyrillic_che_vertstrokeCyrillic_SHHACyrillic_shhaCyrillic_SCHWACyrillic_schwaCyrillic_I_macronCyrillic_i_macronCyrillic_O_barCyrillic_o_barCyrillic_U_macronCyrillic_u_macronArmenian_AYBArmenian_BENArmenian_GIMArmenian_DAArmenian_YECHArmenian_ZAArmenian_EArmenian_ATArmenian_TOArmenian_ZHEArmenian_INIArmenian_LYUNArmenian_KHEArmenian_TSAArmenian_KENArmenian_HOArmenian_DZAArmenian_GHATArmenian_TCHEArmenian_MENArmenian_HIArmenian_NUArmenian_SHAArmenian_VOArmenian_CHAArmenian_PEArmenian_JEArmenian_RAArmenian_SEArmenian_VEVArmenian_TYUNArmenian_REArmenian_TSOArmenian_VYUNArmenian_PYURArmenian_KEArmenian_OArmenian_FEArmenian_apostropheArmenian_accentArmenian_sheshtArmenian_exclamArmenian_amanakArmenian_separation_markArmenian_butArmenian_questionArmenian_paruykArmenian_aybArmenian_benArmenian_gimArmenian_daArmenian_yechArmenian_zaArmenian_eArmenian_atArmenian_toArmenian_zheArmenian_iniArmenian_lyunArmenian_kheArmenian_tsaArmenian_kenArmenian_hoArmenian_dzaArmenian_ghatArmenian_tcheArmenian_menArmenian_hiArmenian_nuArmenian_shaArmenian_voArmenian_chaArmenian_peArmenian_jeArmenian_raArmenian_seArmenian_vevArmenian_tyunArmenian_reArmenian_tsoArmenian_vyunArmenian_pyurArmenian_keArmenian_oArmenian_feArmenian_ligature_ewArmenian_full_stopArmenian_verjaketArmenian_hyphenArmenian_yentamnaArabic_madda_aboveArabic_hamza_aboveArabic_hamza_belowArabic_0Arabic_1Arabic_2Arabic_3Arabic_4Arabic_5Arabic_6Arabic_7Arabic_8Arabic_9Arabic_percentArabic_superscript_alefArabic_ttehArabic_pehArabic_tchehArabic_ddalArabic_rrehArabic_jehArabic_vehArabic_kehehArabic_gafArabic_noon_ghunnaArabic_heh_doachashmeeArabic_heh_goalFarsi_yehArabic_farsi_yehArabic_yeh_bareeArabic_fullstopFarsi_0Farsi_1Farsi_2Farsi_3Farsi_4Farsi_5Farsi_6Farsi_7Farsi_8Farsi_9Sinh_ngSinh_h2Sinh_aSinh_aaSinh_aeSinh_aeeSinh_iSinh_iiSinh_uSinh_uuSinh_riSinh_riiSinh_luSinh_luuSinh_eSinh_eeSinh_aiSinh_oSinh_ooSinh_auSinh_kaSinh_khaSinh_gaSinh_ghaSinh_ng2Sinh_ngaSinh_caSinh_chaSinh_jaSinh_jhaSinh_nyaSinh_jnyaSinh_njaSinh_ttaSinh_tthaSinh_ddaSinh_ddhaSinh_nnaSinh_nddaSinh_thaSinh_thhaSinh_dhaSinh_dhhaSinh_naSinh_ndhaSinh_paSinh_phaSinh_baSinh_bhaSinh_maSinh_mbaSinh_yaSinh_raSinh_laSinh_vaSinh_shaSinh_sshaSinh_saSinh_haSinh_llaSinh_faSinh_alSinh_aa2Sinh_ae2Sinh_aee2Sinh_i2Sinh_ii2Sinh_u2Sinh_uu2Sinh_ru2Sinh_e2Sinh_ee2Sinh_ai2Sinh_o2Sinh_oo2Sinh_au2Sinh_lu2Sinh_ruu2Sinh_luu2Sinh_kunddaliyaGeorgian_anGeorgian_banGeorgian_ganGeorgian_donGeorgian_enGeorgian_vinGeorgian_zenGeorgian_tanGeorgian_inGeorgian_kanGeorgian_lasGeorgian_manGeorgian_narGeorgian_onGeorgian_parGeorgian_zharGeorgian_raeGeorgian_sanGeorgian_tarGeorgian_unGeorgian_pharGeorgian_kharGeorgian_ghanGeorgian_qarGeorgian_shinGeorgian_chinGeorgian_canGeorgian_jilGeorgian_cilGeorgian_charGeorgian_xanGeorgian_jhanGeorgian_haeGeorgian_heGeorgian_hieGeorgian_weGeorgian_harGeorgian_hoeGeorgian_fiBabovedotbabovedotDabovedotdabovedotFabovedotfabovedotLbelowdotlbelowdotMabovedotmabovedotPabovedotpabovedotSabovedotsabovedotTabovedottabovedotWgravewgraveWacutewacuteWdiaeresiswdiaeresisXabovedotxabovedotAbelowdotabelowdotAhookahookAcircumflexacuteacircumflexacuteAcircumflexgraveacircumflexgraveAcircumflexhookacircumflexhookAcircumflextildeacircumflextildeAcircumflexbelowdotacircumflexbelowdotAbreveacuteabreveacuteAbrevegraveabrevegraveAbrevehookabrevehookAbrevetildeabrevetildeAbrevebelowdotabrevebelowdotEbelowdotebelowdotEhookehookEtildeetildeEcircumflexacuteecircumflexacuteEcircumflexgraveecircumflexgraveEcircumflexhookecircumflexhookEcircumflextildeecircumflextildeEcircumflexbelowdotecircumflexbelowdotIhookihookIbelowdotibelowdotObelowdotobelowdotOhookohookOcircumflexacuteocircumflexacuteOcircumflexgraveocircumflexgraveOcircumflexhookocircumflexhookOcircumflextildeocircumflextildeOcircumflexbelowdotocircumflexbelowdotOhornacuteohornacuteOhorngraveohorngraveOhornhookohornhookOhorntildeohorntildeOhornbelowdotohornbelowdotUbelowdotubelowdotUhookuhookUhornacuteuhornacuteUhorngraveuhorngraveUhornhookuhornhookUhorntildeuhorntildeUhornbelowdotuhornbelowdotYgraveygraveYbelowdotybelowdotYhookyhookYtildeytildezerosuperiorfoursuperiorfivesuperiorsixsuperiorsevensuperioreightsuperiorninesuperiorzerosubscriptonesubscripttwosubscriptthreesubscriptfoursubscriptfivesubscriptsixsubscriptsevensubscripteightsubscriptninesubscriptEcuSignColonSignCruzeiroSignFFrancSignLiraSignMillSignNairaSignPesetaSignRupeeSignWonSignNewSheqelSignDongSignpartdifferentialemptysetelementofnotelementofcontainsassquarerootcuberootfourthrootdintegraltintegralbecausenotapproxeqapproxeqnotidenticalstricteqbraille_blankbraille_dots_1braille_dots_2braille_dots_12braille_dots_3braille_dots_13braille_dots_23braille_dots_123braille_dots_4braille_dots_14braille_dots_24braille_dots_124braille_dots_34braille_dots_134braille_dots_234braille_dots_1234braille_dots_5braille_dots_15braille_dots_25braille_dots_125braille_dots_35braille_dots_135braille_dots_235braille_dots_1235braille_dots_45braille_dots_145braille_dots_245braille_dots_1245braille_dots_345braille_dots_1345braille_dots_2345braille_dots_12345braille_dots_6braille_dots_16braille_dots_26braille_dots_126braille_dots_36braille_dots_136braille_dots_236braille_dots_1236braille_dots_46braille_dots_146braille_dots_246braille_dots_1246braille_dots_346braille_dots_1346braille_dots_2346braille_dots_12346braille_dots_56braille_dots_156braille_dots_256braille_dots_1256braille_dots_356braille_dots_1356braille_dots_2356braille_dots_12356braille_dots_456braille_dots_1456braille_dots_2456braille_dots_12456braille_dots_3456braille_dots_13456braille_dots_23456braille_dots_123456braille_dots_7braille_dots_17braille_dots_27braille_dots_127braille_dots_37braille_dots_137braille_dots_237braille_dots_1237braille_dots_47braille_dots_147braille_dots_247braille_dots_1247braille_dots_347braille_dots_1347braille_dots_2347braille_dots_12347braille_dots_57braille_dots_157braille_dots_257braille_dots_1257braille_dots_357braille_dots_1357braille_dots_2357braille_dots_12357braille_dots_457braille_dots_1457braille_dots_2457braille_dots_12457braille_dots_3457braille_dots_13457braille_dots_23457braille_dots_123457braille_dots_67braille_dots_167braille_dots_267braille_dots_1267braille_dots_367braille_dots_1367braille_dots_2367braille_dots_12367braille_dots_467braille_dots_1467braille_dots_2467braille_dots_12467braille_dots_3467braille_dots_13467braille_dots_23467braille_dots_123467braille_dots_567braille_dots_1567braille_dots_2567braille_dots_12567braille_dots_3567braille_dots_13567braille_dots_23567braille_dots_123567braille_dots_4567braille_dots_14567braille_dots_24567braille_dots_124567braille_dots_34567braille_dots_134567braille_dots_234567braille_dots_1234567braille_dots_8braille_dots_18braille_dots_28braille_dots_128braille_dots_38braille_dots_138braille_dots_238braille_dots_1238braille_dots_48braille_dots_148braille_dots_248braille_dots_1248braille_dots_348braille_dots_1348braille_dots_2348braille_dots_12348braille_dots_58braille_dots_158braille_dots_258braille_dots_1258braille_dots_358braille_dots_1358braille_dots_2358braille_dots_12358braille_dots_458braille_dots_1458braille_dots_2458braille_dots_12458braille_dots_3458braille_dots_13458braille_dots_23458braille_dots_123458braille_dots_68braille_dots_168braille_dots_268braille_dots_1268braille_dots_368braille_dots_1368braille_dots_2368braille_dots_12368braille_dots_468braille_dots_1468braille_dots_2468braille_dots_12468braille_dots_3468braille_dots_13468braille_dots_23468braille_dots_123468braille_dots_568braille_dots_1568braille_dots_2568braille_dots_12568braille_dots_3568braille_dots_13568braille_dots_23568braille_dots_123568braille_dots_4568braille_dots_14568braille_dots_24568braille_dots_124568braille_dots_34568braille_dots_134568braille_dots_234568braille_dots_1234568braille_dots_78braille_dots_178braille_dots_278braille_dots_1278braille_dots_378braille_dots_1378braille_dots_2378braille_dots_12378braille_dots_478braille_dots_1478braille_dots_2478braille_dots_12478braille_dots_3478braille_dots_13478braille_dots_23478braille_dots_123478braille_dots_578braille_dots_1578braille_dots_2578braille_dots_12578braille_dots_3578braille_dots_13578braille_dots_23578braille_dots_123578braille_dots_4578braille_dots_14578braille_dots_24578braille_dots_124578braille_dots_34578braille_dots_134578braille_dots_234578braille_dots_1234578braille_dots_678braille_dots_1678braille_dots_2678braille_dots_12678braille_dots_3678braille_dots_13678braille_dots_23678braille_dots_123678braille_dots_4678braille_dots_14678braille_dots_24678braille_dots_124678braille_dots_34678braille_dots_134678braille_dots_234678braille_dots_1234678braille_dots_5678braille_dots_15678braille_dots_25678braille_dots_125678braille_dots_35678braille_dots_135678braille_dots_235678braille_dots_1235678braille_dots_45678braille_dots_145678braille_dots_245678braille_dots_1245678braille_dots_345678braille_dots_1345678braille_dots_2345678braille_dots_12345678hpmute_acutemute_acutehpmute_gravemute_gravehpmute_asciicircummute_asciicircumhpmute_diaeresismute_diaeresishpmute_asciitildemute_asciitildehpliralirahpguilderguilderhpYdiaeresishpIOIOhplongminuslongminushpblockblockDdiaeresisDacute_accentDcedilla_accentDcircumflex_accentDgrave_accentDtildeDring_accentDRemovehpModelock1hpModelock2hpResetResethpSystemSystemhpUserUserhpClearLineClearLinehpInsertLineInsertLinehpDeleteLineDeleteLinehpInsertCharInsertCharhpDeleteCharDeleteCharhpBackTabBackTabhpKP_BackTabKP_BackTabExt16bit_LExt16bit_RosfCopyosfCutosfPasteosfBackTabosfBackSpaceosfClearosfEscapeosfAddModeosfPrimaryPasteosfQuickPasteosfPageLeftosfPageUposfPageDownosfPageRightosfActivateosfMenuBarosfLeftosfUposfRightosfDownosfEndLineosfBeginLineosfEndDataosfBeginDataosfPrevMenuosfNextMenuosfPrevFieldosfNextFieldosfSelectosfInsertosfUndoosfMenuosfCancelosfHelposfSelectAllosfDeselectAllosfReselectosfExtendosfRestoreosfDeleteSunFA_GraveSunFA_CircumSunFA_TildeSunFA_AcuteSunFA_DiaeresisSunFA_CedillaSunF36SunF37SunSys_ReqSunPropsSunFrontSunCopySunOpenSunPasteSunCutSunPowerSwitchSunAudioLowerVolumeSunAudioMuteSunAudioRaiseVolumeSunVideoDegaussSunVideoLowerBrightnessSunVideoRaiseBrightnessSunPowerSwitchShiftXF86MediaPlayPauseXF86ExitXF86AudioBassBoostXF86SportXF86BrightnessAutoXF86MonBrightnessAutoXF86DisplayOffXF86OKXF86GoToXF86InfoXF86VendorLogoXF86MediaSelectProgramGuideXF86MediaSelectHomeXF86MediaLanguageMenuXF86MediaTitleMenuXF86AudioChannelModeXF86AspectRatioXF86MediaSelectPCXF86MediaSelectTVXF86MediaSelectCableXF86MediaSelectVCRXF86MediaSelectVCRPlusXF86MediaSelectSatelliteXF86MediaSelectTapeXF86MediaSelectRadioXF86MediaSelectTunerXF86MediaPlayerXF86MediaSelectTeletextXF86DVDXF86MediaSelectDVDXF86MediaSelectAuxiliaryXF86AudioXF86ChannelUpXF86ChannelDownXF86MediaPlaySlowXF86BreakXF86NumberEntryModeXF86VideoPhoneXF86ZoomResetXF86EditorXF86GraphicsEditorXF86PresentationXF86DatabaseXF86VoicemailXF86AddressbookXF86DisplayToggleXF86SpellCheckXF86ContextMenuXF86MediaRepeatXF8610ChannelsUpXF8610ChannelsDownXF86ImagesXF86NotificationCenterXF86PickupPhoneXF86HangupPhoneXF86LinkPhoneXF86FnXF86Fn_EscXF86Fn_F1XF86Fn_F2XF86Fn_F3XF86Fn_F4XF86Fn_F5XF86Fn_F6XF86Fn_F7XF86Fn_F8XF86Fn_F9XF86Fn_F10XF86Fn_F11XF86Fn_F12XF86Fn_1XF86Fn_2XF86Fn_DXF86Fn_EXF86Fn_FXF86Fn_SXF86Fn_BXF86FnRightShiftXF86Numeric0XF86Numeric1XF86Numeric2XF86Numeric3XF86Numeric4XF86Numeric5XF86Numeric6XF86Numeric7XF86Numeric8XF86Numeric9XF86NumericStarXF86NumericPoundXF86NumericAXF86NumericBXF86NumericCXF86NumericDXF86CameraFocusXF86WPSButtonXF86CameraZoomInXF86CameraZoomOutXF86CameraUpXF86CameraDownXF86CameraLeftXF86CameraRightXF86AttendantOnXF86AttendantOffXF86AttendantToggleXF86LightsToggleXF86ALSToggleXF86RefreshRateToggleXF86ButtonconfigXF86TaskmanagerXF86JournalXF86ControlPanelXF86AppSelectXF86ScreensaverXF86VoiceCommandXF86AssistantXF86EmojiPickerXF86DictateXF86CameraAccessEnableXF86CameraAccessDisableXF86CameraAccessToggleXF86AccessibilityXF86DoNotDisturbXF86BrightnessMinXF86BrightnessMaxXF86KbdInputAssistPrevXF86KbdInputAssistNextXF86KbdInputAssistPrevgroupXF86KbdInputAssistNextgroupXF86KbdInputAssistAcceptXF86KbdInputAssistCancelXF86RightUpXF86RightDownXF86LeftUpXF86LeftDownXF86RootMenuXF86MediaTopMenuXF86Numeric11XF86Numeric12XF86AudioDescXF863DModeXF86NextFavoriteXF86StopRecordXF86PauseRecordXF86VODXF86UnmuteXF86FastReverseXF86SlowReverseXF86DataXF86OnScreenKeyboardXF86PrivacyScreenToggleXF86SelectiveScreenshotXF86NextElementXF86PreviousElementXF86AutopilotEngageToggleXF86MarkWaypointXF86SosXF86NavChartXF86FishingChartXF86SingleRangeRadarXF86DualRangeRadarXF86RadarOverlayXF86TraditionalSonarXF86ClearvuSonarXF86SidevuSonarXF86NavInfoXF86Macro1XF86Macro2XF86Macro3XF86Macro4XF86Macro5XF86Macro6XF86Macro7XF86Macro8XF86Macro9XF86Macro10XF86Macro11XF86Macro12XF86Macro13XF86Macro14XF86Macro15XF86Macro16XF86Macro17XF86Macro18XF86Macro19XF86Macro20XF86Macro21XF86Macro22XF86Macro23XF86Macro24XF86Macro25XF86Macro26XF86Macro27XF86Macro28XF86Macro29XF86Macro30XF86MacroRecordStartXF86MacroRecordStopXF86MacroPresetCycleXF86MacroPreset1XF86MacroPreset2XF86MacroPreset3XF86KbdLcdMenu1XF86KbdLcdMenu2XF86KbdLcdMenu3XF86KbdLcdMenu4XF86KbdLcdMenu5XF86PerformanceModeXF86Switch_VT_1XF86Switch_VT_2XF86Switch_VT_3XF86Switch_VT_4XF86Switch_VT_5XF86Switch_VT_6XF86Switch_VT_7XF86Switch_VT_8XF86Switch_VT_9XF86Switch_VT_10XF86Switch_VT_11XF86Switch_VT_12XF86UngrabXF86ClearGrabXF86Next_VModeXF86Prev_VModeXF86LogWindowTreeXF86LogGrabInfoXF86ModeLockXF86MonBrightnessUpXF86MonBrightnessDownXF86KbdLightOnOffXF86KbdBrightnessUpXF86KbdBrightnessDownXF86MonBrightnessCycleXF86StandbyXF86AudioLowerVolumeXF86AudioMuteXF86AudioRaiseVolumeXF86AudioPlayXF86AudioStopXF86AudioPrevXF86AudioNextXF86HomePageXF86MailXF86StartXF86SearchXF86AudioRecordXF86CalculatorXF86MemoXF86ToDoListXF86CalendarXF86PowerDownXF86ContrastAdjustXF86RockerUpXF86RockerDownXF86RockerEnterXF86BackXF86ForwardXF86StopXF86RefreshXF86PowerOffXF86WakeUpXF86EjectXF86ScreenSaverXF86WWWXF86SleepXF86FavoritesXF86AudioPauseXF86AudioMediaXF86MyComputerXF86VendorHomeXF86LightBulbXF86ShopXF86HistoryXF86OpenURLXF86AddFavoriteXF86HotLinksXF86BrightnessAdjustXF86FinanceXF86CommunityXF86AudioRewindXF86BackForwardXF86Launch0XF86Launch1XF86Launch2XF86Launch3XF86Launch4XF86Launch5XF86Launch6XF86Launch7XF86Launch8XF86Launch9XF86LaunchAXF86LaunchBXF86LaunchCXF86LaunchDXF86LaunchEXF86LaunchFXF86ApplicationLeftXF86ApplicationRightXF86BookXF86CDXF86MediaSelectCDXF86CalculaterXF86ClearXF86CloseXF86CopyXF86CutXF86DisplayXF86DOSXF86DocumentsXF86ExcelXF86ExplorerXF86GameXF86GoXF86iTouchXF86LogOffXF86MarketXF86MeetingXF86MenuKBXF86MenuPBXF86MySitesXF86NewXF86NewsXF86OfficeHomeXF86OpenXF86OptionXF86PasteXF86PhoneXF86QXF86ReplyXF86ReloadXF86RotateWindowsXF86RotationPBXF86RotationKBXF86SaveXF86ScrollUpXF86ScrollDownXF86ScrollClickXF86SendXF86SpellXF86SplitScreenXF86SupportXF86TaskPaneXF86TerminalXF86ToolsXF86TravelXF86UserPBXF86User1KBXF86User2KBXF86VideoXF86WheelButtonXF86WordXF86XferXF86ZoomInXF86ZoomOutXF86AwayXF86MessengerXF86WebCamXF86MailForwardXF86PicturesXF86MusicXF86BatteryXF86BluetoothXF86WLANXF86UWBXF86AudioForwardXF86AudioRepeatXF86AudioRandomPlayXF86SubtitleXF86AudioCycleTrackXF86CycleAngleXF86FrameBackXF86FrameForwardXF86TimeXF86SelectXF86ViewXF86TopMenuXF86RedXF86GreenXF86YellowXF86BlueXF86SuspendXF86HibernateXF86TouchpadToggleXF86TouchpadOnXF86TouchpadOffXF86AudioMicMuteXF86KeyboardXF86WWANXF86RFKillXF86AudioPresetXF86RotationLockToggleXF86FullScreen";

pub(super) static KEYSYM_TO_IDX: PhfMap<u32, u16> = PhfMap {
    key: 12913932095322966823,
    disps: &[(0, 11), (2, 70), (0, 37), (0, 295), (2, 227), (0, 151), (0, 92), (0, 382), (0, 19), (0, 11), (0, 503), (0, 58), (0, 4), (0, 3), (0, 2), (0, 4), (0, 74), (0, 8), (0, 111), (0, 1), (0, 1819), (0, 602), (0, 2488), (0, 707), (1, 1414), (0, 85), (0, 629), (0, 2), (0, 27), (1, 97), (0, 6), (0, 23), (0, 5), (0, 8), (0, 1), (0, 12), (0, 784), (0, 1), (0, 0), (0, 194), (0, 5), (0, 1), (0, 245), (0, 1), (0, 2), (0, 1462), (0, 0), (0, 154), (0, 534), (0, 113), (0, 34), (0, 2), (0, 6), (0, 14), (0, 34), (0, 72), (0, 7), (0, 1831), (0, 1), (0, 8), (0, 3), (0, 2), (0, 168), (0, 43), (0, 0), (0, 2), (0, 964), (0, 0), (0, 1241), (0, 131), (0, 0), (0, 7), (0, 0), (0, 0), (0, 4), (0, 1), (0, 0), (0, 233), (0, 1), (0, 5), (0, 0), (0, 0), (0, 0), (0, 4), (0, 0), (0, 0), (0, 72), (0, 0), (0, 1), (0, 2), (0, 1), (0, 1), (0, 0), (0, 2), (0, 0), (0, 9), (0, 2), (0, 39), (0, 3), (0, 379), (0, 6), (0, 626), (0, 1), (0, 982), (0, 7), (0, 31), (0, 28), (0, 885), (0, 1261), (0, 1485), (0, 1111), (0, 2307), (0, 357), (0, 27), (0, 8), (0, 34), (0, 820), (0, 77), (0, 259), (0, 8), (0, 764), (0, 2369), (0, 529), (4, 1945), (0, 159), (0, 1160), (0, 1860), (0, 214), (0, 31), (0, 360), (0, 42), (0, 214), (0, 8), (0, 1), (0, 3), (0, 16), (0, 2259), (0, 2), (0, 476), (0, 128), (0, 0), (0, 13), (0, 31), (0, 4), (0, 159), (0, 9), (0, 863), (0, 44), (0, 6), (0, 123), (0, 179), (0, 31), (0, 1996), (0, 376), (0, 31), (2, 1566), (0, 2217), (0, 292), (0, 27), (0, 10), (0, 112), (0, 16), (0, 13), (0, 1), (0, 13), (0, 81), (0, 0), (0, 122), (0, 24), (0, 1552), (0, 48), (0, 10), (0, 1818), (0, 6), (2, 2356), (0, 333), (0, 157), (0, 519), (3, 1494), (5, 1933), (0, 298), (0, 347), (0, 298), (0, 8), (0, 0), (0, 36), (0, 8), (0, 367), (0, 5), (0, 37), (0, 5), (0, 1806), (1, 2238), (0, 49), (1, 925), (0, 111), (0, 132), (0, 103), (1, 113), (3, 204), (0, 2), (0, 6), (1, 517), (0, 9), (0, 36), (0, 832), (0, 2), (1, 38), (0, 125), (0, 0), (0, 11), (0, 6), (0, 81), (0, 18), (0, 6), (0, 224), (0, 37), (0, 35), (0, 616), (0, 172), (0, 23), (0, 7), (0, 183), (0, 6), (0, 4), (0, 11), (0, 38), (0, 0), (0, 3), (0, 107), (0, 112), (0, 1), (0, 8), (0, 2129), (0, 7), (0, 0), (0, 871), (0, 3), (0, 0), (0, 617), (0, 8), (0, 1), (0, 65), (0, 0), (0, 0), (0, 90), (0, 980), (2, 941), (0, 20), (0, 1), (0, 2), (0, 2), (0, 1), (0, 2), (0, 10), (0, 0), (0, 533), (0, 2), (0, 2), (0, 1), (0, 12), (0, 0), (0, 159), (0, 6), (0, 3), (1, 207), (0, 42), (0, 239), (0, 336), (0, 213), (0, 21), (0, 8), (0, 0), (0, 7), (0, 84), (0, 27), (0, 993), (0, 70), (0, 7), (1, 1067), (5, 2168), (0, 1917), (0, 1542), (0, 793), (1, 2132), (0, 24), (0, 215), (8, 1656), (0, 95), (0, 70), (0, 3), (0, 705), (0, 741), (5, 1110), (13, 2252), (0, 204), (1, 2115), (0, 8), (0, 143), (9, 1654), (0, 93), (0, 59), (5, 69), (0, 102), (0, 1), (0, 3), (2, 87), (0, 10), (0, 8), (3, 1666), (0, 0), (0, 0), (0, 52), (0, 0), (0, 41), (8, 1175), (0, 62), (0, 24), (0, 29), (0, 6), (0, 186), (0, 23), (0, 1470), (0, 392), (0, 39), (0, 874), (13, 460), (0, 39), (0, 2300), (0, 88), (0, 13), (0, 55), (0, 20), (0, 375), (0, 599), (0, 8), (0, 88), (5, 984), (0, 117), (0, 72), (0, 1895), (4, 892), (0, 405), (0, 1031), (0, 260), (0, 1270), (4, 2079), (0, 120), (0, 247), (0, 117), (15, 1479), (0, 509), (0, 98), (0, 541), (0, 11), (0, 13), (0, 10), (0, 15), (0, 247), (0, 10), (0, 75), (4, 2126), (21, 1824), (0, 277), (8, 2475), (16, 1192), (1, 664), (0, 1003), (0, 84), (5, 458), (0, 239), (0, 5), (0, 344), (3, 389), (0, 3), (0, 617), (2, 412), (0, 4), (0, 1725), (0, 12), (0, 15), (0, 7), (0, 21), (0, 73), (0, 26), (0, 295), (7, 1357), (0, 10), (0, 1991), (0, 731), (0, 15), (0, 217), (6, 2310), (0, 8), (0, 53), (0, 881), (0, 139), (0, 4), (0, 426), (0, 0), (0, 3), (0, 44), (0, 147), (0, 6), (0, 273), (0, 0), (0, 1720), (0, 207), (0, 1), (1, 777), (0, 1487), (0, 3), (0, 1064), (0, 2), (0, 0), (0, 882), (0, 6), (0, 56), (0, 82), (0, 127), (0, 52), (0, 28), (0, 24), (0, 196), (0, 953), (0, 9), (0, 3), (0, 109), (0, 0), (0, 3), (0, 1), (0, 5), (0, 0), (0, 1790), (0, 7), (0, 1), (10, 2304), (0, 4), (0, 160), (15, 542), (0, 458), (0, 4), (0, 52), (0, 0), (0, 24), (0, 1445), (0, 78), (1, 1449), (0, 67), (0, 21), (4, 79), (1, 1664), (13, 851), (0, 55), (1, 2294), (0, 148), (0, 7), (0, 1334), (0, 337), (0, 1414), (0, 0), (0, 1977), (2, 525), (117, 1324), (5, 964), (27, 1285), (1, 1484), (12, 479), (0, 7), (0, 11), (42, 787), (0, 260), (0, 4), (0, 14), (1, 1249), (0, 12), (0, 121), (0, 458), (0, 1381), (0, 271), (0, 44), (0, 308), (0, 3), (0, 13), (0, 10), (0, 2252), (0, 29), (33, 1491), (0, 29), (0, 74), (0, 1163), (0, 304), (2, 761), (33, 1007), (0, 272), (0, 225), (0, 1987), (35, 483), (0, 58), (16, 1653), (2, 96), (0, 15), (0, 183), (0, 3), (0, 116), (0, 475), (0, 1355), (0, 55), (0, 987), (1, 1787), (77, 155), (0, 860)],
    map: &[668, 487, 2368, 208, 453, 2205, 39, 2088, 296, 1188, 1277, 1233, 1476, 957, 1639, 58, 1539, 2555, 1157, 2296, 2502, 676, 983, 484, 1836, 433, 236, 1440, 2475, 839, 2564, 2315, 2621, 267, 457, 1935, 270, 1305, 1803, 1516, 279, 2479, 897, 430, 689, 968, 1010, 900, 685, 2545, 87, 1059, 2391, 681, 836, 2425, 224, 257, 525, 881, 2499, 938, 1652, 2586, 1196, 686, 1808, 631, 2480, 2171, 1773, 526, 1293, 247, 1858, 1365, 618, 878, 14, 10, 2335, 1780, 2448, 1173, 1327, 1854, 507, 1466, 666, 859, 749, 393, 59, 238, 2371, 2503, 1757, 1322, 2407, 1696, 1835, 1189, 901, 1538, 1043, 209, 809, 608, 2525, 599, 2261, 659, 1390, 1441, 2255, 79, 1855, 414, 1079, 803, 136, 2546, 607, 791, 1900, 2606, 2526, 1098, 1497, 37, 2291, 1861, 249, 505, 271, 1577, 1758, 1306, 2464, 661, 396, 115, 1367, 1192, 488, 2587, 2172, 1216, 2522, 1294, 1923, 1421, 2460, 717, 1095, 2443, 56, 1437, 1329, 1217, 2405, 1638, 161, 2552, 394, 1697, 1433, 181, 619, 1138, 15, 1394, 496, 397, 1678, 80, 2029, 157, 1181, 810, 1598, 1147, 1371, 2330, 985, 2456, 1120, 1749, 2010, 1965, 317, 1968, 2262, 1946, 1901, 858, 2298, 1242, 60, 2282, 1919, 38, 1099, 1159, 650, 137, 294, 383, 2007, 2453, 628, 1002, 2605, 1878, 1109, 2116, 751, 1321, 1088, 805, 603, 880, 1178, 2234, 1768, 1495, 452, 182, 2302, 553, 1555, 1140, 1969, 2073, 1698, 999, 1022, 1892, 1728, 162, 113, 707, 116, 1096, 1047, 2094, 2583, 1679, 315, 982, 2030, 2299, 2228, 964, 1902, 339, 986, 1182, 690, 141, 286, 2328, 2011, 1947, 57, 2033, 2512, 24, 2052, 1179, 1883, 1802, 359, 2194, 570, 1076, 2056, 2300, 1992, 285, 307, 1879, 2097, 2021, 1943, 184, 2406, 1011, 1834, 1148, 2213, 1026, 2280, 2117, 2235, 1621, 2053, 2237, 554, 766, 454, 711, 1578, 920, 2173, 2034, 2098, 610, 1569, 1821, 5, 1517, 1579, 1736, 708, 2090, 1602, 1800, 1838, 2395, 2095, 965, 556, 1584, 744, 1776, 573, 262, 1480, 691, 1649, 2101, 782, 2229, 2622, 2249, 1457, 2549, 18, 1714, 308, 1563, 1722, 638, 1152, 384, 2332, 303, 2195, 1540, 2057, 1781, 362, 2153, 981, 772, 459, 745, 924, 2071, 1160, 2212, 1599, 574, 2316, 2046, 2521, 1580, 2279, 1720, 962, 741, 530, 2500, 2076, 2186, 898, 2239, 1928, 1851, 703, 1759, 511, 963, 1250, 2562, 2510, 2074, 1006, 1951, 966, 2373, 882, 485, 1825, 1603, 1619, 1850, 1801, 571, 1653, 1544, 1410, 1641, 670, 1737, 1428, 223, 528, 2345, 4, 1986, 407, 1481, 373, 863, 200, 2115, 1541, 1119, 2523, 2333, 1723, 2408, 1640, 921, 1081, 1904, 1190, 486, 489, 460, 609, 1023, 1198, 2485, 2467, 2610, 2317, 841, 2396, 1218, 1310, 531, 767, 1856, 415, 2411, 1391, 838, 1503, 2607, 2377, 988, 2342, 1852, 899, 1374, 2336, 2327, 1307, 1061, 1501, 250, 1654, 688, 32, 1499, 1458, 2374, 1199, 1219, 1177, 2075, 1295, 1369, 95, 2547, 1525, 671, 864, 1430, 2337, 1622, 1760, 2412, 2430, 1331, 2524, 902, 1052, 1049, 1966, 2623, 2504, 620, 1444, 1804, 837, 2588, 509, 81, 943, 2334, 16, 883, 2567, 83, 2451, 811, 2486, 2611, 635, 2263, 2457, 229, 861, 1392, 1308, 2505, 490, 1450, 227, 887, 318, 2483, 1395, 651, 2608, 2301, 2553, 765, 1375, 2250, 1783, 2392, 662, 251, 667, 117, 792, 1225, 2248, 2431, 103, 20, 1100, 884, 1702, 1339, 2348, 364, 1142, 1062, 248, 807, 2271, 183, 742, 239, 7, 82, 1333, 719, 1124, 1141, 2413, 1445, 2303, 163, 1053, 2625, 186, 1278, 1970, 138, 2624, 62, 1680, 1396, 2267, 639, 340, 409, 595, 142, 1125, 2264, 42, 512, 1882, 2031, 1906, 2372, 541, 1180, 755, 287, 2565, 2386, 1948, 212, 1884, 139, 2397, 1903, 2458, 319, 1451, 408, 2454, 508, 187, 1149, 1865, 784, 557, 2281, 369, 2214, 2201, 2118, 1905, 1143, 1703, 118, 366, 2054, 254, 2174, 416, 1027, 2304, 1521, 1925, 860, 1122, 2099, 276, 1684, 1822, 1434, 168, 86, 1971, 1121, 1746, 1016, 1926, 210, 816, 1952, 2137, 840, 2399, 1681, 1839, 1024, 643, 234, 692, 624, 1748, 947, 1400, 575, 2013, 1184, 11, 1837, 1715, 1949, 1432, 709, 1311, 944, 1165, 1251, 140, 728, 1418, 2058, 1994, 2584, 1071, 40, 1945, 2146, 2193, 370, 1623, 2215, 925, 1044, 1866, 2142, 2119, 558, 343, 1154, 1721, 1704, 1459, 2175, 2055, 2468, 2187, 1028, 367, 1620, 1683, 434, 543, 1993, 1823, 1805, 2423, 1253, 2566, 28, 263, 710, 1522, 1604, 922, 1740, 344, 2017, 1840, 2207, 2550, 341, 1520, 1991, 84, 693, 204, 621, 746, 1161, 1550, 2014, 2527, 1888, 1771, 1724, 764, 2196, 1585, 945, 2081, 2240, 1060, 404, 467, 2185, 2179, 2059, 2269, 1706, 2180, 740, 747, 532, 143, 842, 989, 1162, 1624, 406, 572, 1183, 2347, 2022, 926, 1460, 1764, 2197, 361, 1656, 712, 268, 2378, 1806, 2177, 338, 1237, 1504, 295, 2394, 2188, 1200, 971, 2100, 1254, 673, 1770, 2375, 1565, 114, 1482, 1754, 1463, 1860, 213, 1153, 1742, 1573, 774, 729, 1643, 1716, 865, 2338, 1230, 2571, 923, 1220, 1752, 2346, 1163, 1964, 536, 1586, 1527, 1934, 2077, 948, 649, 652, 1581, 1191, 1605, 590, 1701, 1572, 491, 2401, 533, 2514, 904, 1625, 2319, 1483, 226, 1050, 2501, 1461, 1393, 1542, 2509, 205, 1807, 2148, 506, 1657, 269, 2379, 2343, 514, 228, 640, 1853, 1202, 1376, 843, 1788, 730, 21, 1309, 885, 1655, 2376, 674, 2455, 2590, 1810, 1221, 1487, 1279, 1531, 1409, 479, 241, 1423, 1341, 866, 2414, 1505, 463, 1484, 2217, 1859, 1054, 725, 1335, 2572, 61, 2452, 906, 2609, 1519, 214, 2323, 1009, 1238, 972, 1446, 2626, 63, 1464, 2484, 66, 400, 847, 1725, 2488, 230, 2266, 2459, 905, 44, 1587, 1380, 726, 410, 2352, 418, 586, 85, 1642, 1107, 844, 2533, 1786, 2036, 644, 862, 1785, 50, 622, 1377, 1260, 2568, 492, 1025, 1832, 886, 2466, 788, 813, 119, 2613, 2273, 1067, 41, 1280, 1618, 169, 1343, 2591, 2573, 1493, 2305, 768, 2243, 1077, 641, 1337, 2339, 623, 1972, 1685, 165, 386, 2415, 1582, 1447, 2449, 576, 2078, 889, 1953, 1185, 754, 817, 1063, 2627, 423, 144, 1908, 513, 325, 1452, 1893, 2220, 645, 814, 1381, 435, 1108, 45, 2349, 2199, 753, 1101, 665, 458, 626, 1435, 1927, 192, 1995, 22, 2272, 432, 1389, 1105, 559, 2469, 611, 1166, 120, 1963, 724, 371, 1205, 1617, 1144, 785, 2222, 2143, 1029, 170, 2040, 401, 2104, 1239, 1055, 2433, 123, 88, 1824, 2594, 2037, 1886, 17, 1126, 167, 544, 99, 2120, 697, 1549, 776, 535, 1682, 2018, 2181, 1186, 321, 91, 1667, 145, 2082, 1145, 1912, 2285, 694, 1954, 326, 351, 2015, 748, 2079, 368, 1068, 309, 2598, 1913, 946, 605, 1150, 672, 563, 194, 1083, 1996, 1686, 2309, 824, 363, 2060, 1841, 615, 1967, 2482, 1784, 2210, 927, 1583, 990, 1889, 996, 2105, 1551, 560, 763, 1772, 786, 1155, 713, 1977, 2223, 280, 2477, 1590, 2102, 1984, 950, 70, 311, 1442, 1845, 1765, 2182, 2122, 2556, 1867, 1707, 93, 850, 2270, 2083, 1668, 2019, 2041, 1717, 1976, 495, 1644, 969, 306, 770, 1931, 1407, 98, 2242, 312, 2487, 592, 2064, 1103, 100, 2000, 1588, 2350, 931, 346, 564, 1524, 1570, 304, 2061, 1842, 8, 2139, 19, 510, 949, 1811, 2628, 582, 2320, 614, 323, 465, 967, 272, 1739, 1227, 2306, 1626, 1567, 1743, 437, 1506, 1607, 1658, 2380, 583, 515, 2208, 1846, 1203, 714, 783, 1606, 973, 1164, 812, 1401, 2398, 1206, 992, 1914, 695, 26, 1744, 1226, 1443, 2585, 579, 112, 1890, 867, 1485, 258, 2548, 1408, 471, 1909, 493, 372, 451, 320, 1065, 600, 1726, 545, 593, 1630, 1058, 2324, 1465, 1929, 1629, 1528, 342, 1645, 2353, 207, 189, 819, 1510, 1082, 845, 1552, 2321, 444, 2351, 1812, 993, 698, 907, 2614, 1087, 1955, 585, 253, 277, 419, 1659, 974, 1941, 439, 1809, 1863, 2381, 1507, 2595, 1627, 538, 2450, 1411, 2478, 627, 871, 215, 1257, 2362, 537, 2531, 2437, 1345, 2419, 1592, 1281, 1489, 202, 868, 1790, 1646, 2341, 2508, 2574, 2416, 1486, 473, 2563, 750, 2035, 1315, 1123, 497, 1276, 1932, 908, 2206, 2354, 991, 2325, 1789, 1204, 1312, 2, 2493, 494, 424, 1453, 456, 970, 231, 815, 732, 2615, 646, 2385, 2530, 1382, 2592, 1529, 2554, 420, 2253, 104, 1296, 1766, 2209, 2382, 2596, 846, 211, 2470, 3, 519, 2534, 1066, 2159, 1767, 1427, 190, 65, 2532, 2438, 1259, 1069, 872, 1208, 1347, 1422, 1426, 2481, 216, 1523, 242, 2490, 869, 2417, 612, 1424, 1262, 402, 92, 1316, 2489, 171, 2420, 2575, 149, 350, 653, 1533, 252, 345, 417, 233, 1240, 462, 1787, 777, 1003, 1488, 425, 146, 820, 1064, 2619, 1085, 1313, 2286, 1042, 848, 1106, 794, 961, 1383, 1608, 421, 1936, 1168, 1070, 2038, 2283, 1110, 2597, 731, 1167, 2151, 125, 1034, 2589, 632, 2310, 2441, 1997, 2244, 561, 2161, 727, 578, 1869, 2254, 2106, 1349, 46, 1031, 2155, 1978, 71, 2535, 2224, 546, 195, 723, 124, 218, 654, 69, 1012, 1553, 1688, 775, 687, 2039, 821, 2311, 2124, 2612, 849, 787, 151, 1868, 2084, 1956, 1669, 1282, 2511, 1156, 316, 2307, 2020, 1127, 2529, 2429, 1872, 147, 699, 1826, 696, 888, 222, 1687, 1937, 580, 1169, 2528, 932, 903, 377, 1399, 1827, 2176, 27, 2062, 1035, 1870, 1998, 1111, 550, 1557, 2618, 1873, 929, 2103, 176, 1975, 1090, 2236, 1032, 1692, 1406, 2225, 565, 601, 353, 1979, 547, 2238, 1745, 2065, 101, 2293, 1000, 2024, 1689, 51, 715, 89, 2211, 1013, 2043, 2126, 23, 955, 2593, 598, 1151, 2085, 1891, 1844, 2551, 1957, 1402, 399, 2287, 289, 2200, 1554, 1670, 1885, 1379, 581, 2066, 2557, 1195, 2426, 498, 1575, 2002, 2541, 1664, 933, 1727, 172, 1847, 1828, 2178, 2569, 2132, 1036, 2063, 1813, 68, 551, 336, 930, 1558, 310, 1355, 1568, 616, 288, 1973, 2108, 111, 298, 474, 1718, 596, 102, 975, 1609, 733, 265, 440, 716, 1508, 1660, 436, 994, 1944, 1419, 1794, 1490, 1403, 1999, 956, 264, 2363, 637, 1730, 29, 1373, 2570, 1412, 1462, 1708, 1791, 1534, 1593, 1960, 636, 1193, 1471, 736, 1647, 2340, 953, 1700, 203, 1228, 2580, 2080, 1911, 1468, 1665, 934, 1769, 2355, 327, 909, 1631, 2221, 2538, 1266, 2001, 2326, 734, 1511, 2042, 892, 1613, 521, 2513, 1990, 1546, 2292, 2461, 979, 1297, 2491, 2616, 700, 442, 2439, 976, 2494, 1509, 1731, 2383, 1974, 1661, 219, 1610, 1286, 1, 1910, 1814, 1491, 1249, 1648, 678, 1762, 1930, 2579, 1512, 873, 217, 2506, 629, 1283, 2421, 1741, 1535, 1561, 870, 987, 789, 2402, 1589, 196, 499, 1194, 2107, 1317, 2576, 469, 1848, 1004, 2218, 1041, 475, 1818, 2356, 2016, 1086, 1413, 388, 191, 2359, 398, 122, 2149, 1275, 426, 893, 188, 90, 2387, 2492, 832, 305, 1537, 1594, 1298, 1384, 2440, 48, 1448, 220, 255, 682, 1236, 2473, 127, 851, 2384, 1782, 795, 1287, 2617, 1265, 2536, 2495, 324, 874, 1314, 2517, 2274, 952, 534, 664, 910, 245, 1284, 1351, 2418, 2157, 47, 630, 173, 2284, 411, 914, 25, 389, 2577, 503, 2252, 655, 2360, 1056, 1795, 387, 1896, 822, 591, 520, 1318, 243, 504, 2436, 669, 1089, 2518, 1128, 1302, 2150, 385, 427, 72, 647, 49, 148, 2559, 584, 773, 2256, 52, 1385, 1938, 2471, 106, 1449, 1170, 2462, 1072, 675, 2246, 2276, 412, 1211, 129, 648, 1288, 1115, 1359, 2165, 375, 201, 177, 2308, 349, 779, 328, 76, 2312, 197, 232, 984, 1693, 2604, 2163, 1871, 1353, 2602, 1397, 829, 174, 313, 334, 2268, 480, 237, 2089, 718, 625, 73, 1690, 1792, 2313, 448, 2128, 1033, 634, 153, 2445, 518, 235, 1916, 1958, 2251, 329, 1174, 2086, 2202, 2409, 527, 2288, 2364, 1671, 799, 1116, 1763, 2067, 1864, 1171, 1113, 330, 121, 2322, 2003, 379, 1210, 2277, 995, 1245, 1862, 1829, 1037, 2112, 778, 1005, 378, 376, 951, 2422, 2294, 2230, 562, 780, 1398, 133, 2109, 1920, 31, 1018, 1719, 337, 356, 549, 1981, 1694, 291, 64, 1556, 2025, 1897, 2045, 1134, 1112, 1887, 1710, 1015, 2144, 1547, 43, 1750, 1404, 1962, 75, 2198, 1675, 2023, 1014, 2145, 1175, 2087, 1559, 1672, 1709, 2044, 152, 890, 1092, 1959, 758, 826, 756, 954, 2004, 2068, 566, 660, 737, 2295, 1666, 1571, 1255, 935, 2048, 1632, 2444, 2113, 597, 300, 1038, 178, 2231, 552, 1815, 290, 450, 916, 1614, 2465, 1591, 1241, 1357, 1820, 355, 476, 1705, 1732, 828, 1796, 589, 1662, 704, 1611, 1875, 1933, 198, 1777, 759, 771, 2189, 1091, 1158, 2026, 1548, 2434, 679, 701, 1711, 2365, 2110, 1595, 282, 757, 1988, 1492, 1874, 517, 567, 274, 1636, 939, 738, 656, 1473, 446, 855, 2331, 481, 642, 1830, 1472, 522, 2072, 977, 852, 911, 936, 2190, 1414, 477, 2496, 1633, 542, 2357, 2515, 1816, 1774, 2147, 1030, 894, 942, 1405, 2216, 2388, 1755, 1793, 833, 1514, 1615, 1819, 1733, 1526, 529, 1797, 1663, 683, 221, 891, 1532, 2369, 1299, 1007, 1415, 1469, 2403, 1751, 1048, 347, 1980, 875, 206, 680, 2366, 1513, 273, 1596, 1576, 959, 2581, 523, 633, 1285, 482, 390, 915, 2203, 1416, 2404, 1474, 94, 856, 1989, 540, 735, 1987, 2049, 958, 374, 1075, 1207, 912, 431, 2329, 1319, 762, 928, 2358, 1438, 1387, 853, 2463, 2497, 980, 428, 1425, 1051, 895, 225, 1057, 130, 2389, 1917, 2543, 2257, 1073, 834, 1386, 797, 2032, 2542, 684, 2600, 2370, 34, 1961, 2427, 1849, 2560, 2361, 2558, 1467, 2167, 1361, 1924, 876, 1290, 1894, 246, 2400, 2275, 6, 413, 594, 2446, 1562, 677, 2520, 391, 2245, 96, 175, 1612, 657, 1248, 1212, 1323, 1187, 2539, 1320, 1429, 1530, 500, 2474, 606, 2424, 2367, 769, 1778, 275, 185, 1940, 154, 2519, 1117, 2578, 830, 1008, 2620, 429, 1304, 760, 2393, 2289, 1454, 2227, 801, 1898, 2544, 1074, 1301, 2258, 1114, 380, 1431, 35, 256, 1436, 1939, 1439, 2516, 74, 131, 1214, 2601, 1264, 1378, 2169, 77, 199, 752, 360, 617, 1564, 331, 793, 1172, 2599, 1921, 1019, 109, 1325, 1982, 2141, 158, 292, 357, 1135, 1950, 395, 1388, 392, 781, 2091, 2027, 2247, 2447, 1676, 78, 1300, 2472, 155, 1046, 13, 2260, 1132, 1363, 1292, 2008, 1118, 1176, 134, 1985, 1080, 1880, 332, 1129, 1918, 1455, 790, 802, 539, 283, 1695, 1146, 2069, 2134, 2290, 381, 36, 1078, 1536, 2219, 132, 1477, 1761, 1097, 1899, 2114, 1691, 879, 54, 2050, 1831, 2012, 180, 917, 1478, 1712, 997, 179, 1922, 913, 348, 1094, 2111, 1775, 297, 1136, 160, 2410, 1420, 705, 1983, 299, 1566, 301, 244, 2092, 1017, 1876, 1677, 107, 30, 998, 1843, 1093, 2130, 1877, 587, 53, 260, 1020, 702, 2009, 2278, 1674, 2259, 1209, 2028, 259, 2435, 568, 739, 940, 164, 613, 1130, 284, 2006, 1942, 1637, 358, 382, 1560, 2070, 1634, 302, 266, 12, 1756, 1039, 2232, 2226, 721, 2191, 2507, 1817, 1713, 2442, 1881, 55, 501, 524, 1515, 720, 1616, 2051, 2233, 2432, 2183, 1021, 1734, 2096, 1045, 2540, 1650, 1600, 1753, 278, 2603, 1628, 403, 2093, 1798, 314, 663, 1224, 1470, 918, 588, 516, 1494, 135, 1303, 502, 2297, 2204, 33, 1597, 1518, 706, 1779, 1215, 761, 960, 941, 604, 405, 1475, 2005, 1699, 1857, 1417, 857, 569, 1001, 1915, 937, 97, 2192, 1635, 1456, 2498, 2314, 2561, 1133, 1833, 293, 2582, 722, 555, 108, 1895, 1907, 483, 835, 896, 919, 1738, 335, 2390, 240, 658, 105, 2344, 1673, 2047, 1799, 1213, 1729, 1601, 1651, 1735, 2184, 978, 1747, 1131, 854, 743, 2318, 156, 2428, 1040, 0, 261, 2476, 877, 1479],
    _phantom: core::marker::PhantomData,
};

pub(super) static NAME_TO_IDX: PhfMap<[u8], u16> = PhfMap {
    key: 12913932095322966823,
    disps: &[(0, 5), (0, 0), (0, 2), (0, 1), (0, 14), (0, 0), (0, 14), (0, 0), (0, 14), (0, 2), (0, 5), (0, 32), (0, 10), (0, 0), (0, 2), (0, 4), (0, 13), (0, 5), (0, 24), (0, 19), (0, 5), (0, 20), (0, 0), (0, 5), (0, 63), (0, 1), (0, 0), (0, 0), (0, 1), (0, 9), (0, 2), (0, 6), (0, 0), (0, 0), (0, 0), (0, 2), (0, 6), (0, 7), (0, 1), (0, 48), (0, 0), (0, 6), (0, 0), (0, 7), (0, 5), (0, 50), (0, 5), (0, 1), (0, 0), (0, 0), (0, 0), (0, 0), (0, 16), (0, 0), (0, 19), (0, 13), (0, 1), (0, 1), (0, 35), (0, 0), (0, 0), (0, 7), (0, 19), (0, 74), (0, 5), (0, 38), (0, 37), (0, 3), (0, 0), (0, 6), (0, 22), (0, 1), (0, 20), (0, 0), (0, 1), (0, 8), (0, 35), (0, 1), (0, 12), (0, 0), (0, 9), (0, 7), (0, 1), (0, 6), (0, 15), (0, 0), (0, 1), (0, 0), (0, 0), (0, 0), (0, 51), (0, 10), (0, 0), (0, 2), (0, 31), (0, 35), (0, 27), (0, 12), (0, 20), (0, 3), (0, 40), (0, 0), (0, 1), (0, 7), (0, 28), (0, 0), (0, 65), (0, 7), (0, 0), (0, 0), (0, 0), (0, 1), (0, 16), (0, 0), (0, 2), (0, 3), (0, 3), (0, 24), (0, 0), (0, 0), (0, 10), (0, 48), (0, 4), (0, 0), (0, 9), (0, 0), (0, 0), (0, 9), (0, 5), (0, 0), (0, 13), (0, 0), (0, 17), (0, 42), (0, 0), (0, 2), (0, 1), (0, 1), (0, 22), (0, 9), (0, 11), (0, 1), (0, 0), (0, 0), (0, 8), (0, 0), (0, 5), (0, 32), (0, 10), (0, 28), (0, 3), (0, 19), (0, 6), (0, 6), (0, 150), (0, 82), (0, 15), (0, 2), (0, 0), (0, 1), (0, 0), (0, 1), (0, 2), (0, 2), (0, 0), (0, 8), (0, 9), (0, 0), (0, 16), (0, 3), (0, 35), (0, 51), (0, 11), (0, 0), (0, 1), (0, 47), (0, 5), (0, 2), (0, 7), (0, 0), (0, 2), (0, 35), (0, 1), (0, 48), (0, 0), (0, 1), (0, 5), (0, 19), (0, 12), (0, 17), (0, 2), (0, 1), (0, 2), (0, 1), (0, 12), (0, 0), (0, 29), (0, 1), (0, 1), (0, 17), (0, 0), (0, 31), (0, 1), (0, 6), (0, 1), (0, 7), (0, 0), (0, 2), (0, 2), (0, 0), (0, 0), (0, 54), (0, 1), (0, 22), (0, 0), (0, 50), (0, 0), (0, 0), (0, 120), (0, 0), (0, 0), (0, 3), (0, 1), (0, 19), (0, 2), (0, 15), (0, 67), (0, 21), (0, 15), (0, 5), (0, 18), (0, 12), (0, 0), (0, 8), (0, 0), (0, 7), (0, 4), (0, 0), (0, 2), (0, 5), (0, 4), (0, 2), (0, 6), (0, 1), (0, 0), (0, 0), (0, 3), (0, 25), (0, 2), (0, 17), (0, 2), (0, 19), (0, 39), (0, 0), (0, 23), (0, 0), (0, 34), (0, 17), (0, 18), (0, 0), (0, 0), (0, 2), (0, 0), (0, 21), (0, 43), (0, 34), (0, 8), (0, 3), (0, 4), (0, 470), (0, 12), (0, 6), (0, 2), (0, 56), (0, 466), (0, 35), (0, 2), (0, 7), (0, 0), (0, 28), (0, 3), (0, 26), (0, 0), (0, 1), (0, 24), (0, 13), (0, 3), (0, 0), (0, 12), (0, 113), (0, 2), (0, 15), (0, 7), (0, 0), (0, 0), (0, 0), (0, 3), (0, 11), (0, 0), (0, 8), (0, 1), (0, 36), (0, 30), (0, 9), (0, 1), (0, 58), (0, 0), (0, 0), (0, 1), (0, 16), (0, 2), (0, 30), (0, 4), (0, 24), (0, 8), (0, 4), (0, 0), (0, 2), (0, 0), (0, 8), (0, 1), (0, 1), (0, 21), (0, 7), (0, 22), (0, 0), (0, 225), (0, 59), (0, 1), (0, 11), (0, 2), (0, 2), (0, 0), (0, 20), (0, 12), (0, 17), (0, 52), (0, 86), (0, 4), (0, 7), (0, 0), (0, 75), (0, 5), (0, 0), (0, 0), (0, 0), (0, 0), (0, 7), (0, 23), (0, 7), (0, 11), (0, 0), (0, 6), (0, 6), (0, 151), (0, 64), (0, 14), (0, 14), (0, 10), (0, 0), (0, 57), (0, 15), (0, 87), (0, 39), (0, 0), (0, 7), (0, 3), (0, 9), (0, 115), (0, 0), (0, 0), (0, 37), (0, 6), (0, 0), (0, 54), (0, 13), (0, 2), (0, 55), (0, 0), (0, 1), (0, 0), (0, 3), (0, 3), (0, 10), (1, 1), (0, 35), (0, 9), (0, 73), (0, 2), (0, 124), (0, 4), (0, 35), (0, 10), (0, 75), (0, 0), (0, 72), (0, 68), (0, 1), (0, 19), (0, 2), (0, 5), (0, 68), (0, 0), (0, 324), (0, 15), (0, 17), (0, 17), (0, 1), (0, 183), (0, 1), (0, 16), (0, 3), (0, 13), (0, 180), (0, 22), (0, 298), (0, 6), (0, 90), (0, 8), (0, 77), (0, 0), (0, 7), (0, 4), (0, 13), (0, 0), (0, 5), (0, 9), (0, 0), (0, 0), (0, 30), (0, 2), (0, 1), (0, 35), (0, 27), (0, 0), (0, 70), (0, 27), (0, 2), (0, 0), (0, 96), (0, 16), (0, 43), (0, 5), (0, 0), (0, 2), (0, 4), (0, 28), (0, 1), (0, 536), (0, 4), (0, 0), (0, 1), (0, 16), (0, 5), (0, 5), (0, 56), (0, 7), (0, 9), (0, 125), (0, 14), (0, 12), (0, 12), (0, 1), (0, 8), (0, 12), (0, 9), (0, 6), (0, 59), (0, 15), (0, 20), (0, 3), (0, 13), (0, 25), (0, 2), (0, 9), (0, 15), (0, 183), (0, 3), (0, 0), (0, 0), (0, 0), (0, 28), (0, 46), (0, 21), (0, 0), (0, 15), (0, 0), (0, 3), (0, 25), (0, 3), (0, 19), (0, 0), (0, 18), (0, 6), (0, 20), (0, 1), (0, 25), (0, 0), (0, 123), (0, 21), (0, 43), (0, 18), (0, 22), (0, 6), (0, 25), (0, 64), (0, 6), (0, 3), (0, 0), (0, 0), (0, 8), (0, 61), (0, 121), (0, 8), (0, 35), (0, 0), (0, 0), (0, 260), (0, 32), (0, 24), (0, 22), (0, 113), (0, 36), (0, 0), (0, 4), (0, 6), (0, 205), (0, 84), (0, 5), (0, 21), (0, 5), (0, 59), (0, 8), (0, 26), (0, 6), (0, 25), (0, 2), (0, 0), (0, 41), (0, 9), (0, 51), (0, 21), (0, 0), (0, 1), (0, 1), (0, 1), (0, 0), (0, 4), (0, 10), (0, 10), (0, 0), (0, 50), (0, 1), (0, 1), (0, 0), (0, 2), (0, 219), (0, 0), (0, 72), (0, 1), (0, 0), (0, 26), (0, 0), (0, 0), (0, 16), (0, 234), (0, 471), (0, 152), (0, 314), (0, 21), (0, 20), (0, 19), (0, 222), (0, 0), (0, 44), (0, 0), (0, 45), (0, 81), (0, 2), (0, 17), (0, 44), (0, 14), (0, 33), (0, 20), (0, 4), (0, 16), (0, 10), (0, 86), (0, 37), (0, 0), (0, 312), (0, 0), (0, 0), (0, 3), (0, 43), (0, 1), (0, 167), (0, 8), (0, 0), (0, 3), (0, 17), (0, 6), (0, 1), (0, 142), (0, 0), (0, 626), (0, 627), (0, 2), (0, 496), (0, 123), (0, 209), (0, 47), (0, 751), (0, 15), (0, 49), (0, 19), (0, 7), (0, 32), (0, 0), (0, 1), (0, 453), (0, 0), (0, 13), (0, 39), (0, 8), (0, 5), (0, 6), (0, 1), (0, 0), (0, 756), (0, 19), (0, 7), (0, 0), (0, 1), (0, 12), (0, 1), (0, 5), (0, 81), (0, 36), (0, 17), (0, 135), (0, 0), (0, 10), (0, 7), (0, 0), (0, 13), (0, 11), (0, 516), (0, 10), (0, 21), (0, 613), (0, 22), (0, 84), (0, 0), (0, 60), (0, 4), (0, 0), (0, 6), (0, 4), (0, 1), (0, 4), (0, 16), (0, 10), (0, 565), (0, 3), (0, 8), (0, 5), (0, 30), (0, 3), (0, 2), (0, 29), (0, 1), (0, 26), (0, 2), (0, 58), (0, 186), (0, 114), (0, 1), (0, 0), (0, 24), (0, 20), (0, 100), (0, 0), (0, 127), (0, 197), (0, 55), (0, 4), (0, 750), (0, 10), (0, 5), (0, 5), (0, 4), (0, 141), (0, 0), (0, 66), (0, 462), (0, 245), (0, 9), (0, 73), (0, 0), (0, 9), (0, 293), (0, 0), (0, 17), (0, 37), (0, 67), (0, 6), (0, 14), (0, 0), (0, 7), (0, 171), (0, 9), (0, 176), (0, 470), (0, 13), (0, 493), (0, 11), (0, 156), (0, 0), (0, 69), (0, 60), (0, 6), (0, 48), (0, 40), (0, 21), (0, 8), (0, 0), (0, 0), (0, 10), (0, 15), (0, 273), (0, 131), (0, 146), (0, 819), (0, 0), (0, 6), (0, 265), (0, 0), (0, 116), (0, 0), (0, 13), (0, 62), (0, 241), (0, 24), (0, 8), (0, 41), (0, 832), (0, 229), (0, 23), (0, 0), (0, 102), (0, 596), (0, 93), (0, 871), (0, 447), (0, 2), (0, 20), (0, 74), (0, 0), (0, 38), (0, 414), (0, 84), (0, 42), (0, 0), (0, 9), (0, 0), (0, 889), (0, 1), (0, 5), (0, 18), (0, 79), (0, 25), (0, 0), (0, 60), (0, 1003), (0, 60), (0, 3), (0, 287), (0, 56), (0, 394), (0, 292), (0, 280), (0, 53), (0, 1), (0, 13), (0, 68), (0, 40), (0, 672), (0, 13), (0, 248), (0, 31), (0, 4), (0, 1), (0, 26), (0, 173), (0, 43), (0, 0), (0, 119), (0, 3), (0, 135), (0, 5), (0, 7), (0, 2), (0, 6), (0, 15), (0, 8), (0, 634), (0, 1), (0, 11), (0, 131), (0, 6), (0, 65), (0, 1), (0, 0), (0, 238), (0, 247), (0, 532), (0, 678), (0, 0), (0, 82), (0, 3), (0, 88), (0, 0), (0, 409), (0, 0), (0, 7), (0, 4), (0, 80), (0, 1), (0, 29), (0, 19), (0, 39), (0, 0), (0, 0), (0, 7), (0, 4), (0, 24), (0, 181), (0, 3), (0, 490), (0, 305), (0, 1), (0, 3), (0, 9), (0, 13), (0, 1), (0, 119), (0, 13), (0, 222), (0, 6), (0, 68), (0, 41), (0, 1), (0, 0), (0, 271), (0, 31), (0, 15), (0, 2), (0, 17), (0, 195), (0, 5), (0, 6), (0, 6), (0, 15), (0, 13), (0, 0), (0, 0), (0, 19), (0, 1), (0, 3), (0, 271), (0, 3), (1, 28), (0, 70), (0, 0), (0, 0), (0, 41), (0, 3), (0, 1), (0, 807), (0, 968), (0, 10), (0, 0), (0, 0), (0, 307), (0, 146), (0, 235), (0, 782), (0, 14), (0, 10), (0, 0), (0, 135), (0, 1), (0, 1155), (0, 155), (0, 27), (0, 68), (0, 498), (0, 45), (0, 38), (0, 10), (0, 13), (0, 2), (0, 415), (0, 1124), (0, 1), (0, 1379), (0, 0), (0, 0), (0, 136), (0, 24), (0, 0), (0, 13), (0, 29), (0, 92), (0, 60), (0, 18), (0, 1876), (0, 44), (0, 0), (0, 0), (0, 5), (0, 46), (0, 1317), (0, 262), (0, 6), (0, 195), (0, 0), (0, 1550), (0, 0), (0, 2100), (0, 4), (0, 53), (0, 19), (0, 22), (0, 0), (0, 170), (0, 181), (0, 28), (0, 0), (0, 75), (0, 0), (0, 65), (0, 32), (0, 602), (0, 118), (0, 130), (0, 3), (0, 0), (0, 0), (0, 0), (0, 29), (0, 927), (0, 17), (0, 1961), (0, 1681), (0, 216), (0, 12), (0, 1232), (0, 146), (0, 0), (0, 0), (0, 0), (0, 37), (0, 1052), (0, 0), (0, 6), (0, 2424), (0, 47), (0, 0), (0, 420), (0, 58), (0, 1), (0, 29), (0, 0), (0, 1), (0, 13), (0, 12), (0, 8), (0, 92), (0, 69), (0, 1565), (0, 13), (0, 269), (0, 39), (0, 0), (0, 1282), (0, 0), (0, 33), (0, 23), (0, 45), (0, 214), (0, 11), (0, 1352), (0, 1397), (0, 2332), (0, 9), (0, 117), (0, 21), (0, 568), (0, 6), (0, 23), (0, 0), (0, 1588), (0, 238), (0, 15), (0, 6), (0, 84), (0, 1623), (0, 0), (0, 822), (0, 326), (0, 3), (0, 26), (0, 11), (0, 3), (0, 7), (0, 27), (0, 3), (0, 4), (0, 47), (1, 1), (0, 27), (0, 731)],
    map: &[1657, 102, 340, 1023, 2233, 1430, 742, 2468, 2037, 1935, 231, 727, 1842, 1029, 2401, 1392, 32, 1546, 1361, 1967, 422, 1876, 1007, 2151, 165, 1377, 2491, 2442, 185, 952, 823, 461, 888, 881, 2316, 242, 1985, 359, 1003, 1899, 1614, 1306, 2563, 1957, 1646, 1179, 1913, 1554, 1597, 1991, 104, 2010, 1251, 28, 2331, 1885, 1527, 457, 2287, 71, 677, 1431, 2199, 1249, 1380, 401, 483, 689, 1073, 619, 204, 199, 2190, 1697, 271, 1761, 2206, 317, 982, 1592, 1833, 1262, 2154, 190, 1849, 272, 966, 43, 73, 172, 2222, 1506, 434, 1322, 1508, 1569, 1159, 1528, 2291, 920, 1919, 588, 1517, 1016, 2444, 1533, 566, 1939, 223, 521, 950, 1366, 1248, 2579, 1107, 864, 2591, 942, 1316, 728, 90, 1673, 2013, 1279, 1474, 2568, 589, 558, 1526, 1028, 91, 1454, 140, 1692, 2611, 2202, 2162, 1653, 1243, 548, 869, 296, 368, 2527, 1436, 2068, 2342, 1997, 2313, 2413, 2088, 1185, 1033, 2454, 109, 1759, 857, 1416, 2421, 424, 2387, 2251, 676, 476, 267, 1389, 812, 696, 1639, 1626, 2266, 1765, 1429, 1421, 1926, 581, 2212, 146, 640, 1473, 79, 1740, 745, 1482, 1131, 2083, 789, 1897, 488, 657, 2091, 1848, 553, 2548, 1126, 1244, 1405, 2213, 1496, 2359, 2144, 2449, 120, 1819, 1065, 352, 1311, 1266, 1685, 366, 1942, 1620, 2456, 239, 902, 512, 1534, 288, 2511, 101, 1634, 1433, 1782, 828, 1578, 2388, 2120, 1374, 540, 1650, 2185, 901, 1205, 164, 1337, 2322, 2530, 1518, 2564, 1875, 506, 2066, 1151, 1953, 2056, 2232, 1164, 2249, 1747, 1864, 2373, 2416, 1390, 1928, 808, 1863, 1343, 260, 2118, 112, 286, 1197, 1485, 1478, 297, 1817, 908, 621, 1941, 868, 97, 2382, 1261, 1411, 2375, 948, 2538, 1231, 793, 2556, 1006, 2598, 867, 1711, 1596, 1549, 68, 2384, 733, 335, 1119, 404, 433, 110, 1412, 971, 1045, 2015, 1360, 209, 1208, 1917, 111, 583, 1326, 2184, 174, 1117, 435, 1803, 162, 544, 250, 25, 92, 2357, 2057, 2215, 1398, 612, 556, 679, 703, 651, 1418, 1229, 1432, 839, 1175, 646, 1187, 437, 668, 186, 425, 56, 2182, 1196, 387, 1825, 1666, 2217, 2187, 1856, 2076, 1110, 720, 325, 372, 680, 824, 350, 1468, 2044, 1732, 1011, 1365, 2575, 2476, 775, 1303, 803, 715, 2595, 467, 1069, 2223, 904, 155, 291, 1327, 704, 782, 1469, 673, 541, 1081, 2336, 2028, 594, 113, 431, 2314, 2161, 1854, 1531, 332, 2248, 1861, 2326, 365, 2599, 1137, 2080, 2348, 1250, 1486, 2201, 1964, 1594, 2306, 1556, 2189, 0, 2197, 1263, 1212, 1336, 282, 1846, 1269, 1170, 735, 2518, 481, 2009, 2093, 1406, 1145, 1258, 235, 573, 2235, 961, 2399, 939, 2426, 1687, 2381, 1912, 1570, 1046, 1267, 1632, 897, 1356, 699, 159, 2493, 587, 1591, 1333, 884, 2303, 1845, 1221, 2305, 1786, 10, 954, 645, 2129, 1891, 2195, 1720, 2446, 1206, 1966, 1658, 1739, 130, 1947, 2543, 1089, 2261, 478, 2340, 2127, 292, 206, 755, 2084, 2255, 1980, 493, 964, 840, 1998, 596, 2350, 2329, 157, 1101, 339, 2439, 27, 2549, 1453, 1498, 949, 936, 1410, 1766, 1859, 1387, 2411, 1866, 1585, 1901, 750, 188, 653, 2045, 1031, 1171, 348, 1297, 2423, 752, 2374, 58, 1850, 114, 2457, 222, 1055, 1916, 1444, 1135, 2547, 534, 293, 2094, 732, 1287, 1224, 290, 1095, 414, 1027, 1235, 81, 1428, 1037, 1467, 468, 1462, 1132, 1219, 1054, 985, 1852, 2626, 2156, 11, 543, 1729, 132, 45, 1784, 845, 516, 2497, 2485, 2107, 508, 2506, 870, 765, 822, 579, 2529, 2176, 153, 299, 247, 2224, 1290, 2552, 598, 1172, 1959, 471, 2330, 1667, 200, 2069, 1905, 1768, 1996, 2474, 992, 804, 277, 161, 644, 2064, 743, 1019, 821, 2393, 2089, 1077, 967, 1609, 2167, 2560, 134, 2079, 266, 1320, 2532, 1347, 1150, 2571, 1427, 2265, 1860, 1706, 1886, 208, 1715, 2016, 2310, 2607, 1893, 210, 1201, 1879, 1452, 273, 1278, 1191, 151, 701, 2407, 426, 2343, 960, 2220, 1500, 2526, 2465, 903, 1277, 1523, 522, 1058, 1969, 2153, 1551, 308, 492, 1458, 1424, 173, 649, 2461, 708, 307, 9, 2038, 1785, 1423, 1239, 976, 1308, 2371, 2605, 1100, 417, 249, 911, 1993, 1372, 1270, 816, 2085, 380, 1505, 1446, 1000, 166, 1449, 1461, 1677, 838, 1907, 625, 221, 1323, 1758, 42, 807, 643, 475, 253, 2436, 1995, 211, 582, 470, 347, 1257, 740, 1362, 847, 2264, 655, 1129, 2181, 1955, 2256, 882, 623, 988, 1289, 865, 230, 1435, 1479, 316, 753, 1368, 1738, 564, 83, 1617, 1369, 385, 778, 1840, 507, 2469, 561, 1442, 2128, 2026, 2438, 2542, 1744, 12, 229, 2131, 107, 1798, 330, 1684, 511, 255, 1183, 2141, 246, 1111, 486, 227, 1310, 1734, 2174, 70, 1702, 956, 1724, 2268, 1305, 1009, 181, 2349, 143, 1385, 1952, 770, 1203, 1465, 747, 2252, 635, 1881, 890, 827, 1238, 656, 1572, 420, 3, 1542, 1712, 714, 787, 1020, 874, 663, 2428, 899, 1352, 2513, 1984, 1625, 2138, 523, 1329, 2370, 2126, 937, 2544, 1762, 1522, 1575, 2448, 1511, 314, 495, 328, 1012, 47, 749, 1683, 2338, 2227, 632, 1704, 959, 88, 396, 1910, 2221, 1674, 1284, 1222, 2462, 269, 2022, 226, 2602, 2531, 430, 281, 1808, 2509, 2301, 413, 575, 1010, 2194, 2100, 2230, 799, 306, 2404, 997, 1604, 2327, 1525, 984, 659, 1393, 2043, 791, 405, 2545, 1830, 1978, 631, 1233, 1835, 1911, 371, 278, 2288, 2417, 116, 2335, 60, 2008, 809, 1480, 376, 1647, 182, 576, 1695, 1965, 351, 1540, 465, 2562, 1603, 142, 1286, 2385, 1638, 910, 2124, 1413, 1867, 1986, 836, 133, 1108, 1822, 344, 532, 1401, 62, 472, 1530, 1144, 148, 1664, 1488, 1601, 1877, 519, 1213, 1573, 354, 1072, 149, 1457, 1619, 593, 1562, 880, 36, 1153, 1324, 2168, 2443, 1815, 2178, 1211, 1174, 1746, 2000, 1934, 1535, 2247, 1162, 452, 615, 1565, 2308, 66, 18, 1124, 1507, 923, 363, 1818, 2554, 1539, 545, 2246, 411, 463, 2590, 5, 2420, 1472, 1001, 1440, 391, 1106, 275, 754, 1075, 756, 921, 1118, 237, 2620, 189, 2505, 1804, 126, 197, 337, 2275, 1202, 1138, 256, 1042, 2441, 1903, 1564, 160, 620, 1036, 763, 1112, 1757, 2025, 2196, 2400, 410, 2234, 1335, 1553, 1382, 192, 1109, 2520, 647, 725, 1307, 1519, 1102, 953, 1082, 1218, 15, 183, 941, 2475, 1114, 641, 601, 129, 327, 2321, 877, 1334, 1477, 1403, 2405, 2135, 1255, 1719, 970, 2369, 320, 1140, 1654, 2082, 1025, 2597, 86, 1742, 364, 1872, 1701, 537, 1354, 716, 2048, 2395, 517, 2295, 592, 1699, 262, 2, 72, 2334, 1066, 1448, 1753, 2304, 44, 883, 2603, 2332, 875, 2273, 1315, 1447, 2351, 1264, 1364, 539, 686, 382, 2270, 1851, 1983, 85, 2132, 2001, 1794, 734, 1276, 515, 388, 1847, 2440, 1220, 531, 2011, 1816, 758, 580, 2594, 1103, 2613, 2152, 2130, 739, 1797, 1979, 1160, 462, 1811, 2029, 1948, 1623, 2471, 321, 1494, 2453, 1294, 2323, 1857, 402, 503, 2422, 694, 935, 2606, 1157, 759, 916, 567, 1713, 205, 979, 1741, 1929, 1167, 999, 398, 2615, 614, 2498, 955, 1497, 565, 2522, 284, 1628, 2243, 1513, 1871, 1463, 2570, 1812, 1076, 1217, 49, 1751, 1938, 477, 1348, 2567, 176, 633, 2109, 1963, 584, 713, 2587, 2101, 769, 1756, 2020, 926, 963, 559, 1022, 1373, 2092, 2355, 1809, 1068, 1559, 2149, 131, 1700, 2062, 831, 191, 1889, 1878, 551, 1659, 67, 1139, 2450, 378, 171, 1954, 1063, 259, 1247, 2274, 1338, 1487, 2023, 2155, 990, 2095, 978, 2240, 353, 705, 2415, 825, 1828, 2137, 1581, 2368, 2122, 606, 117, 1726, 1301, 345, 1391, 137, 1887, 1921, 2036, 1946, 393, 1896, 525, 1225, 1560, 1059, 1071, 331, 2186, 245, 1827, 922, 1760, 2402, 1349, 818, 613, 69, 65, 1383, 341, 177, 2516, 862, 2058, 2136, 38, 2473, 1778, 1342, 1265, 318, 1725, 1696, 1293, 243, 319, 2090, 1668, 1048, 1612, 1328, 403, 1635, 448, 1599, 2496, 367, 1080, 1125, 834, 943, 1568, 924, 1838, 329, 2277, 432, 568, 2378, 688, 1972, 608, 1451, 2096, 1409, 1032, 983, 139, 416, 1064, 2572, 2566, 2437, 730, 526, 1275, 634, 20, 445, 2271, 2372, 788, 1660, 2121, 268, 2312, 446, 2280, 389, 528, 1529, 1093, 2019, 1663, 1641, 2164, 447, 2262, 806, 1115, 1176, 2134, 1806, 2358, 1381, 1141, 454, 51, 2459, 975, 533, 1214, 1717, 2624, 194, 383, 1375, 1999, 2484, 1057, 1743, 607, 2053, 74, 295, 1309, 1521, 1292, 1918, 1123, 1439, 195, 1122, 484, 859, 1563, 1555, 2517, 1773, 889, 2140, 1455, 1799, 2464, 1920, 1503, 861, 2490, 2061, 224, 1924, 1087, 301, 2452, 669, 801, 2379, 1252, 2553, 1873, 2183, 774, 1710, 1484, 773, 2160, 2211, 1331, 458, 270, 1992, 1971, 2254, 1676, 744, 46, 662, 886, 2569, 2492, 843, 2171, 1004, 2319, 1795, 1127, 1200, 334, 813, 441, 1313, 2236, 630, 885, 1350, 2394, 1915, 835, 1147, 2499, 1464, 933, 722, 2054, 1982, 2623, 2297, 1680, 907, 1280, 2339, 2279, 2165, 772, 1193, 1014, 394, 480, 1415, 201, 2188, 436, 138, 1330, 994, 700, 1230, 2193, 1094, 932, 2377, 1586, 2610, 96, 415, 2455, 1843, 1988, 2536, 360, 1226, 451, 2065, 1686, 2424, 2432, 2324, 1690, 2524, 675, 315, 1537, 1489, 2072, 2208, 2514, 487, 768, 1524, 2289, 674, 783, 866, 2241, 1359, 1722, 1165, 1026, 2507, 2467, 280, 1344, 2099, 35, 1973, 274, 1629, 217, 1655, 1890, 1078, 912, 2237, 1735, 1775, 1777, 1631, 1384, 1177, 346, 1062, 810, 449, 1994, 1253, 1092, 741, 610, 14, 59, 2383, 198, 76, 2356, 2390, 2403, 336, 498, 1826, 1271, 99, 1750, 1408, 1084, 2512, 225, 549, 785, 846, 1675, 2601, 1937, 604, 851, 815, 2012, 1678, 1633, 2017, 61, 2039, 878, 609, 1437, 1583, 395, 746, 2205, 1425, 379, 1870, 1396, 1567, 1792, 2003, 1116, 1853, 369, 2392, 1321, 1774, 1298, 509, 2055, 108, 595, 650, 1783, 1772, 2073, 691, 1610, 1353, 310, 1018, 622, 2347, 1705, 1855, 1121, 900, 684, 892, 1538, 257, 491, 2345, 2228, 618, 1194, 1288, 2282, 167, 1902, 1791, 40, 1017, 1606, 1589, 1332, 2276, 1682, 1831, 1149, 2546, 2309, 2214, 1636, 636, 800, 450, 1821, 629, 505, 141, 616, 2272, 1091, 1923, 466, 2116, 2582, 1181, 1098, 2408, 2086, 1602, 915, 428, 1013, 830, 658, 2482, 729, 2296, 406, 2435, 2409, 1801, 1130, 917, 128, 1688, 87, 2117, 1402, 1689, 2596, 2589, 2075, 1314, 1607, 780, 30, 1210, 552, 1260, 2558, 84, 797, 946, 1618, 1557, 2041, 848, 1099, 1008, 784, 214, 158, 456, 390, 313, 529, 2577, 1580, 2578, 965, 2018, 1259, 1561, 2198, 777, 1363, 1858, 2050, 1520, 2014, 2119, 187, 957, 972, 361, 2311, 853, 1085, 2021, 298, 2519, 2463, 1083, 514, 1495, 2317, 1898, 520, 150, 757, 671, 1637, 1932, 324, 2625, 1426, 721, 666, 578, 1475, 895, 1925, 1989, 1021, 527, 2559, 837, 1030, 233, 2002, 798, 175, 1178, 1945, 1195, 1709, 2063, 572, 726, 665, 2299, 2510, 1133, 1283, 2242, 22, 2169, 479, 1215, 664, 887, 1545, 1459, 2172, 1051, 909, 1237, 1593, 1319, 464, 2614, 1346, 1571, 304, 1730, 2115, 1839, 89, 2515, 1652, 934, 1481, 1053, 2146, 876, 2561, 31, 4, 1661, 571, 855, 1681, 1787, 2353, 356, 1395, 2365, 711, 48, 1414, 792, 2042, 557, 1781, 2081, 178, 2290, 1627, 1904, 2581, 1914, 1960, 323, 2429, 2278, 1285, 264, 154, 1120, 1716, 2110, 1974, 207, 236, 2361, 702, 1509, 2472, 2354, 555, 1869, 485, 1209, 1163, 707, 1388, 2628, 1752, 1865, 2030, 384, 1622, 1002, 2609, 1182, 37, 1748, 63, 738, 1050, 2431, 1152, 1079, 627, 228, 591, 100, 1958, 1608, 891, 1931, 1754, 309, 660, 2414, 1223, 2143, 1922, 1976, 216, 78, 1662, 2108, 929, 1510, 1483, 2269, 1386, 796, 2477, 1434, 333, 1643, 2111, 106, 1038, 766, 442, 1642, 1299, 1493, 33, 1532, 2495, 2031, 1737, 989, 2406, 538, 2049, 1796, 1450, 1813, 2550, 19, 285, 1577, 2293, 302, 871, 1834, 1466, 2458, 2386, 2292, 1471, 2112, 2114, 2097, 1723, 1161, 577, 2363, 981, 1810, 2612, 2204, 829, 469, 698, 2180, 2419, 896, 1640, 683, 1341, 254, 305, 248, 2352, 343, 312, 2147, 1968, 2170, 1228, 50, 115, 820, 1950, 429, 1358, 2451, 1714, 718, 1404, 1649, 1134, 64, 392, 135, 105, 1829, 670, 617, 1956, 16, 2218, 2565, 1990, 1470, 1566, 706, 914, 196, 127, 991, 1516, 1339, 1698, 1793, 123, 2535, 893, 1940, 2040, 1550, 1600, 602, 913, 1245, 1515, 1302, 1805, 1155, 2298, 1611, 2177, 639, 2478, 974, 218, 1113, 947, 762, 2077, 145, 2145, 2133, 124, 2320, 1168, 311, 289, 2258, 1874, 1615, 358, 1041, 1024, 386, 2508, 2159, 1771, 2585, 854, 118, 419, 2619, 1097, 930, 2551, 826, 652, 2618, 672, 692, 958, 1, 1367, 2586, 2533, 1708, 202, 1146, 795, 55, 2593, 2163, 2627, 24, 761, 2244, 499, 381, 421, 1246, 1888, 2239, 2521, 2158, 1790, 179, 2503, 212, 1788, 322, 170, 1884, 945, 856, 2470, 2487, 802, 697, 373, 1630, 605, 1357, 2325, 737, 2257, 1820, 1256, 1192, 2060, 1180, 562, 1312, 1707, 1906, 2460, 667, 125, 2307, 2346, 156, 2483, 2391, 500, 1169, 152, 2362, 2005, 928, 2150, 2504, 2027, 554, 1184, 1764, 342, 2225, 1158, 842, 1142, 1671, 2592, 75, 370, 1558, 276, 399, 1422, 357, 1317, 2260, 690, 1645, 542, 2588, 1399, 1672, 764, 252, 2033, 1052, 938, 1769, 2051, 1340, 1951, 490, 998, 709, 2445, 860, 771, 234, 1128, 362, 29, 546, 1582, 1837, 1188, 2555, 2337, 1272, 1679, 1883, 440, 1086, 1961, 811, 203, 215, 872, 144, 1836, 2576, 26, 1576, 1216, 94, 1841, 2494, 122, 2102, 1648, 1776, 2573, 7, 2425, 1832, 1868, 1514, 1351, 2412, 1933, 53, 844, 951, 2034, 2396, 1721, 39, 1613, 2557, 1044, 2046, 2098, 790, 687, 535, 2537, 563, 2071, 1823, 1962, 1587, 678, 1780, 261, 760, 2231, 2534, 2604, 103, 685, 2229, 1789, 279, 776, 502, 1198, 80, 2216, 1204, 2209, 2004, 232, 1407, 338, 1400, 98, 2540, 1895, 1274, 1154, 21, 1136, 1616, 1207, 2294, 2226, 1770, 1034, 1090, 1927, 1088, 2059, 723, 1254, 2501, 585, 460, 283, 586, 377, 349, 2341, 2245, 8, 2267, 1476, 850, 1579, 1232, 2113, 987, 1543, 496, 786, 2541, 1504, 817, 919, 2434, 925, 1242, 1236, 2210, 1595, 682, 240, 2617, 1060, 879, 52, 1621, 1015, 833, 1345, 444, 418, 2622, 482, 977, 93, 455, 2580, 968, 603, 1035, 1445, 2104, 858, 459, 1900, 638, 905, 2318, 2207, 397, 1273, 1670, 570, 1909, 2433, 1588, 510, 1930, 1981, 1892, 1490, 560, 2035, 1394, 1598, 1304, 1056, 1975, 163, 2398, 1552, 590, 2283, 1590, 489, 2300, 1105, 1949, 2502, 219, 501, 1281, 1656, 2367, 1802, 1767, 2179, 731, 873, 2175, 294, 693, 996, 2574, 1779, 710, 2200, 1049, 169, 1318, 1492, 355, 2500, 244, 2238, 1547, 1325, 1512, 2006, 973, 1397, 918, 2488, 518, 2480, 661, 2024, 724, 1669, 1189, 1814, 2366, 2389, 1300, 326, 263, 1371, 1755, 1718, 695, 1824, 931, 241, 648, 1491, 1936, 1862, 1731, 2360, 2192, 1844, 1419, 1703, 781, 1987, 1376, 624, 814, 2523, 2219, 574, 17, 474, 2007, 944, 1691, 2074, 1074, 1665, 57, 2078, 1156, 712, 1043, 1370, 2032, 375, 2259, 2087, 453, 1061, 927, 2621, 2125, 1694, 1970, 407, 1460, 1651, 136, 719, 1977, 748, 1644, 473, 2067, 1443, 1296, 1502, 1800, 995, 2427, 2173, 1070, 1047, 303, 2157, 438, 1745, 626, 1005, 819, 220, 1733, 1040, 119, 751, 1727, 513, 1291, 1420, 494, 1944, 2600, 1199, 1417, 1227, 1548, 1544, 986, 898, 863, 2281, 852, 497, 77, 504, 1894, 1268, 13, 147, 2191, 409, 180, 1763, 34, 2253, 2364, 906, 524, 794, 1584, 530, 2447, 2203, 1096, 2584, 2479, 1456, 1295, 1378, 412, 736, 637, 1943, 2302, 95, 213, 2616, 1282, 2333, 962, 193, 6, 2285, 2430, 1355, 2528, 2106, 2328, 2376, 1536, 2166, 969, 642, 2466, 767, 258, 2418, 600, 2105, 1880, 1241, 779, 1234, 2047, 2123, 1574, 423, 251, 265, 1240, 2583, 980, 168, 400, 2142, 832, 1067, 1438, 569, 547, 654, 1736, 2315, 1190, 805, 41, 2103, 1039, 849, 82, 2139, 2539, 1693, 2284, 287, 184, 439, 1186, 1541, 1728, 2397, 2263, 550, 536, 1441, 121, 1104, 1908, 2148, 1807, 940, 2286, 599, 1624, 1501, 1499, 628, 2250, 597, 2608, 2380, 23, 238, 2525, 2481, 1173, 1882, 1749, 1166, 2070, 717, 2410, 443, 2486, 1379, 1148, 54, 2052, 1605, 408, 1143, 841, 2489, 2344, 427, 374, 300, 681, 611, 993, 894],
    _phantom: core::marker::PhantomData,
};

pub(super) static LOWER_NAME_TO_IDX: PhfMap<[u8], u16> = PhfMap {
    key: 12913932095322966823,
    disps: &[(0, 276), (0, 17), (0, 50), (0, 136), (0, 1), (0, 60), (0, 9), (0, 106), (0, 272), (0, 2), (0, 119), (0, 72), (0, 2), (0, 52), (0, 332), (0, 84), (0, 167), (0, 8), (0, 6), (0, 232), (0, 55), (0, 0), (0, 22), (0, 37), (0, 27), (0, 101), (0, 85), (0, 0), (0, 60), (0, 0), (0, 3), (0, 95), (0, 2), (0, 5), (0, 359), (0, 3), (0, 1), (0, 10), (0, 123), (0, 148), (0, 21), (0, 6), (0, 1), (0, 0), (0, 4), (0, 243), (0, 20), (0, 14), (0, 29), (0, 41), (0, 1), (0, 28), (0, 237), (0, 187), (0, 32), (0, 233), (0, 29), (0, 1), (0, 46), (0, 38), (0, 567), (0, 144), (0, 77), (0, 22), (0, 383), (0, 3), (0, 32), (0, 186), (0, 49), (0, 182), (0, 257), (0, 193), (0, 19), (0, 4), (0, 197), (0, 157), (0, 117), (0, 4), (0, 4), (0, 567), (0, 94), (0, 706), (0, 28), (0, 397), (0, 74), (0, 0), (0, 229), (0, 0), (0, 32), (0, 645), (0, 4), (0, 8), (0, 13), (0, 6), (0, 768), (0, 347), (0, 31), (0, 9), (0, 2), (0, 284), (0, 70), (0, 21), (0, 3), (0, 6), (0, 8), (0, 41), (0, 177), (0, 454), (0, 33), (0, 5), (0, 0), (0, 37), (0, 26), (0, 1), (0, 19), (0, 151), (0, 192), (0, 0), (0, 51), (0, 0), (0, 11), (0, 241), (0, 278), (0, 135), (0, 0), (0, 0), (0, 148), (0, 18), (0, 0), (0, 9), (0, 241), (0, 180), (0, 22), (0, 304), (0, 1), (0, 0), (0, 15), (0, 9), (0, 221), (0, 1501), (0, 82), (0, 109), (0, 23), (0, 28), (0, 0), (0, 88), (0, 705), (0, 2), (0, 115), (0, 132), (0, 7), (0, 18), (0, 167), (0, 24), (0, 233), (0, 398), (0, 0), (0, 5), (0, 5), (0, 47), (0, 8), (0, 1), (0, 52), (0, 0), (0, 7), (0, 3), (0, 155), (0, 0), (0, 16), (0, 49), (0, 66), (0, 73), (0, 727), (0, 1), (1, 16), (0, 148), (0, 204), (0, 14), (0, 282), (0, 6), (0, 453), (0, 306), (0, 9), (1, 825), (0, 592), (0, 18), (0, 0), (0, 3), (0, 4), (0, 0), (0, 67), (0, 252), (0, 57), (0, 26), (0, 165), (0, 1), (0, 1), (0, 47), (0, 71), (0, 1), (0, 517), (0, 1), (0, 75), (0, 344), (0, 631), (0, 32), (0, 1), (0, 2), (0, 62), (0, 433), (0, 27), (0, 7), (0, 336), (0, 93), (0, 1247), (0, 8), (0, 347), (0, 0), (0, 25), (0, 259), (0, 25), (0, 52), (0, 0), (0, 1), (0, 16), (0, 7), (0, 297), (0, 180), (0, 2), (0, 1101), (0, 79), (0, 3), (0, 465), (0, 53), (0, 212), (0, 5), (0, 12), (0, 94), (0, 249), (0, 719), (0, 188), (0, 11), (0, 1642), (0, 1323), (0, 86), (0, 950), (0, 0), (0, 52), (0, 668), (0, 114), (0, 0), (0, 22), (0, 1283), (0, 0), (0, 16), (0, 16), (0, 1196), (0, 1), (0, 206), (0, 1), (0, 6), (0, 89), (0, 15), (0, 1), (0, 251), (0, 1530), (0, 220), (0, 97), (0, 6), (0, 34), (0, 0), (0, 1), (0, 10), (0, 38), (0, 904), (0, 0), (0, 121), (0, 98), (0, 9), (0, 25), (0, 0), (0, 13), (0, 223), (0, 178), (0, 25), (1, 89), (0, 4), (0, 10), (0, 464), (0, 11), (0, 1), (0, 336), (0, 58), (0, 4), (0, 77), (0, 353), (0, 7), (0, 68), (0, 88), (0, 259), (0, 545), (0, 1152), (0, 12), (0, 261), (0, 307), (0, 367), (0, 8), (0, 187), (0, 26), (0, 239), (0, 73), (0, 1025), (0, 0), (0, 0), (0, 42), (0, 0), (0, 1473), (0, 47), (0, 12), (0, 17), (0, 3), (0, 181), (0, 144), (0, 132), (0, 313), (0, 0), (0, 11), (0, 51), (0, 38), (0, 5), (0, 916), (0, 16), (0, 237), (0, 3), (0, 3), (0, 250), (0, 32), (0, 733), (0, 573), (0, 0), (0, 30), (0, 69), (0, 345), (0, 136), (0, 1511), (0, 15), (0, 431), (0, 14), (0, 2), (0, 491), (0, 60), (0, 466), (0, 109), (0, 104), (0, 69), (0, 1), (0, 44), (0, 8), (0, 16), (0, 15), (0, 1457), (0, 569), (0, 746), (0, 283), (0, 1793), (0, 219), (0, 833), (0, 564), (0, 1189), (0, 52), (0, 164), (0, 0), (0, 7), (0, 196), (0, 41), (0, 41), (0, 29), (0, 404), (0, 17), (0, 17), (0, 1), (0, 2005), (0, 40), (0, 8), (0, 434), (2, 1455), (0, 356), (1, 1004), (0, 395), (0, 25), (0, 359), (1, 543), (0, 750), (0, 2), (0, 208), (0, 0), (0, 74), (0, 1095), (0, 1921), (0, 472), (0, 8), (0, 1378), (0, 851), (0, 385), (0, 795), (0, 52), (1, 155), (0, 296), (1, 1336), (0, 46), (0, 1054), (1, 1850), (0, 1335), (0, 206), (0, 1367), (0, 797), (0, 80), (0, 9), (0, 247), (0, 27), (0, 147), (0, 113), (0, 901), (0, 2159), (0, 1482), (0, 27), (0, 208), (0, 77), (0, 1), (0, 258), (0, 39), (0, 0), (0, 471), (0, 131), (0, 0), (0, 11), (0, 478), (0, 18), (0, 897), (0, 672), (0, 7), (0, 812), (0, 1), (0, 763), (0, 2), (0, 28), (0, 0), (0, 3), (0, 1815), (0, 2264), (0, 7), (0, 13), (1, 836), (0, 2), (2, 2001), (0, 2200), (0, 1200), (0, 5), (0, 1089), (0, 227), (0, 24), (0, 6), (0, 3), (4, 356), (0, 2), (0, 202), (0, 88), (0, 11), (0, 6), (0, 336), (0, 242), (0, 214), (0, 275), (0, 286), (1, 331), (0, 935), (0, 41), (1, 671), (2, 1612), (0, 1891), (4, 1478), (2, 467), (0, 714), (0, 40), (0, 3), (0, 140), (0, 6), (0, 26), (0, 6), (0, 587), (0, 18), (0, 2), (0, 993), (0, 9), (0, 5), (0, 800), (0, 1530), (0, 742), (0, 202), (0, 45), (0, 38), (0, 5), (0, 12), (0, 105), (0, 603), (7, 1419), (0, 17), (0, 2), (0, 0), (0, 150), (0, 8), (0, 31)],
    map: &[1657, 986, 2320, 1985, 2479, 2145, 2042, 1823, 1599, 1064, 1051, 1205, 2530, 426, 1003, 881, 809, 1844, 1029, 2429, 190, 2054, 2428, 1921, 2355, 2222, 1807, 2175, 1425, 1098, 243, 1043, 2335, 903, 2590, 768, 1580, 2007, 702, 1550, 1653, 962, 857, 754, 1650, 942, 2119, 1346, 2100, 1410, 1018, 1370, 913, 326, 1951, 1260, 2544, 1679, 834, 2326, 1211, 735, 1337, 1715, 886, 737, 202, 981, 2226, 376, 1853, 830, 279, 1455, 1956, 2539, 2311, 2351, 1421, 311, 1383, 2529, 771, 872, 2340, 2133, 697, 1554, 2362, 1942, 1084, 1846, 487, 1121, 854, 1437, 2290, 1230, 1647, 1944, 1052, 1193, 1028, 1578, 2231, 96, 904, 1317, 649, 921, 860, 1359, 852, 98, 1656, 2225, 743, 923, 2328, 2442, 1156, 1319, 1787, 2199, 1703, 2051, 715, 1154, 1138, 2214, 1641, 1369, 1208, 1169, 1160, 2323, 1723, 868, 180, 1584, 1781, 1828, 118, 869, 2597, 485, 409, 1894, 1244, 734, 926, 1256, 2194, 2274, 1854, 825, 2281, 2275, 1598, 2566, 2277, 1707, 620, 1941, 2385, 2035, 601, 814, 430, 2298, 1010, 641, 445, 2563, 877, 1945, 2423, 657, 1365, 1114, 199, 1056, 911, 2049, 1215, 1962, 405, 722, 976, 712, 785, 2553, 1313, 488, 2278, 1988, 397, 745, 709, 2593, 1858, 989, 1684, 2022, 1373, 2103, 2162, 900, 220, 2554, 295, 355, 1190, 1269, 2456, 104, 1610, 2230, 1344, 1515, 707, 1054, 2392, 322, 2558, 1187, 955, 1429, 254, 378, 1570, 719, 494, 644, 1243, 1819, 1582, 2248, 677, 334, 1392, 2434, 871, 1567, 2243, 13, 2027, 2240, 654, 1926, 2140, 634, 756, 1183, 794, 1817, 2368, 15, 741, 1059, 1348, 1863, 1039, 2155, 1115, 613, 1439, 175, 876, 2224, 2410, 192, 1527, 2014, 2618, 2295, 727, 1859, 982, 961, 717, 2546, 1640, 1242, 862, 596, 1025, 866, 1351, 2272, 1390, 1167, 678, 2552, 1247, 2598, 2165, 2381, 444, 795, 2081, 2086, 974, 669, 764, 1352, 1044, 1015, 2099, 1701, 1665, 1840, 2390, 2435, 2495, 867, 345, 320, 1530, 1725, 1835, 1620, 2431, 554, 2084, 739, 2497, 1562, 1508, 2338, 2251, 957, 1233, 1918, 212, 1253, 1510, 1841, 726, 1281, 2304, 2406, 1388, 797, 2060, 1258, 2238, 2366, 2229, 1909, 372, 116, 1164, 2150, 858, 293, 2555, 1609, 2421, 2354, 1375, 2437, 242, 1279, 1321, 2471, 1830, 2333, 493, 2227, 1633, 1659, 1112, 2577, 1209, 1171, 425, 1419, 1525, 2596, 2026, 1217, 1539, 1268, 2196, 1568, 563, 1602, 1943, 1356, 645, 914, 1286, 1130, 392, 1420, 369, 2317, 733, 1122, 2161, 1014, 2408, 661, 277, 382, 480, 2331, 1867, 1142, 1521, 1658, 1547, 2002, 1223, 1327, 501, 1191, 972, 2138, 2315, 973, 1350, 1673, 979, 1586, 2628, 1532, 799, 1615, 1125, 2571, 1721, 1705, 1323, 1927, 928, 1512, 2608, 1897, 3, 183, 1545, 365, 437, 1593, 1298, 2402, 905, 665, 2604, 736, 436, 484, 1541, 611, 1534, 2066, 1495, 129, 1435, 1902, 455, 174, 1182, 491, 1078, 1536, 1042, 2468, 2595, 2503, 2203, 265, 2487, 1162, 1924, 632, 1683, 2616, 1649, 2560, 2474, 2521, 558, 1412, 769, 316, 600, 559, 840, 845, 122, 1451, 753, 1511, 173, 1053, 34, 1402, 870, 1799, 1607, 2206, 1952, 1986, 1953, 371, 2284, 2207, 863, 1140, 1878, 1380, 2591, 1229, 1516, 380, 332, 2088, 1262, 948, 2220, 2509, 2404, 393, 29, 4, 694, 1661, 2585, 1674, 2578, 2058, 114, 2583, 2182, 1795, 184, 1136, 624, 1692, 833, 103, 1969, 1175, 1376, 1222, 2073, 716, 690, 1263, 1655, 1629, 1954, 2622, 639, 1860, 1431, 2191, 312, 2476, 178, 2515, 999, 157, 248, 2564, 117, 127, 827, 967, 2561, 2327, 1057, 1519, 1801, 2439, 1576, 1964, 565, 1573, 266, 698, 1900, 506, 218, 1445, 1975, 508, 1068, 310, 2626, 1631, 439, 636, 1961, 1173, 2606, 2377, 1126, 2263, 171, 327, 2156, 625, 1433, 2453, 1978, 1639, 627, 1914, 2034, 108, 1132, 2620, 1998, 2493, 1118, 2486, 224, 2282, 2420, 638, 2549, 2477, 1214, 2432, 2361, 260, 2096, 1540, 1123, 561, 748, 1071, 1331, 1339, 1066, 252, 308, 1107, 1779, 443, 2115, 653, 1509, 1966, 808, 652, 2202, 368, 1005, 2255, 2149, 629, 2525, 1531, 2587, 1073, 1441, 1034, 1574, 1076, 1318, 2141, 264, 883, 846, 668, 1970, 2394, 1757, 2459, 1668, 17, 1255, 1920, 1974, 2334, 1185, 2296, 692, 592, 1102, 656, 1971, 2112, 889, 497, 2195, 2358, 2079, 2469, 292, 2603, 2023, 2059, 2121, 1502, 1361, 2018, 2256, 1963, 1128, 2038, 703, 2367, 2441, 2396, 1329, 787, 1876, 377, 1906, 932, 191, 386, 1535, 2462, 113, 1083, 890, 197, 1447, 2319, 2234, 2153, 2579, 349, 2089, 1765, 662, 1861, 2254, 1274, 2097, 798, 1652, 249, 84, 1305, 2025, 843, 1832, 1236, 2506, 16, 618, 1277, 1170, 2409, 1643, 126, 2120, 1623, 1686, 1595, 1958, 1382, 946, 1290, 330, 2387, 1669, 1842, 873, 557, 1821, 1225, 169, 1644, 1820, 2418, 1565, 69, 2063, 490, 762, 453, 2159, 1973, 1379, 481, 1017, 1153, 2480, 951, 786, 1065, 2108, 290, 2612, 2557, 222, 1275, 897, 8, 1505, 2485, 1677, 2137, 346, 1338, 347, 2189, 884, 2357, 1234, 2365, 2341, 894, 1500, 1908, 2436, 2348, 1691, 1890, 2241, 2048, 1597, 2581, 1316, 31, 1700, 2412, 2172, 2183, 2292, 395, 1976, 431, 2266, 1295, 1272, 2321, 2625, 602, 1642, 1303, 124, 1309, 1294, 1157, 2071, 2169, 2211, 1622, 280, 1322, 2184, 1019, 1249, 1805, 2262, 2067, 997, 2213, 1093, 2151, 1769, 1687, 983, 1689, 1733, 1325, 879, 937, 606, 2405, 1335, 2010, 1278, 1895, 1159, 595, 2414, 1882, 2291, 564, 2004, 2075, 10, 2609, 28, 642, 1094, 195, 1685, 1809, 1960, 1815, 449, 648, 216, 2205, 1210, 1793, 1336, 125, 2501, 1994, 2289, 1145, 0, 1088, 2216, 2300, 1038, 2288, 275, 2147, 495, 929, 603, 1934, 1006, 1395, 1564, 110, 179, 1315, 749, 689, 778, 1907, 81, 2322, 492, 1070, 1699, 1785, 853, 79, 1713, 1196, 2126, 674, 502, 1847, 954, 2610, 7, 1559, 1896, 250, 23, 65, 1979, 2186, 324, 2061, 1332, 12, 1312, 341, 966, 1938, 2185, 1082, 791, 1884, 1977, 836, 1161, 1965, 721, 892, 1063, 664, 94, 21, 2021, 813, 675, 1264, 752, 720, 396, 2033, 960, 1041, 422, 1058, 1232, 2562, 2276, 200, 2116, 1288, 181, 780, 1079, 1872, 92, 2050, 1189, 1749, 2037, 1328, 509, 875, 2457, 1624, 1604, 1381, 2280, 2268, 1117, 1497, 2154, 403, 2403, 2253, 800, 941, 767, 314, 1930, 824, 1120, 822, 2388, 666, 2285, 2359, 2369, 364, 2570, 1176, 428, 763, 1206, 1870, 820, 2110, 1220, 2265, 75, 1201, 2244, 1948, 413, 708, 849, 291, 2492, 1922, 790, 121, 2228, 223, 779, 1625, 2041, 2573, 2545, 1245, 1199, 406, 1141, 555, 1866, 424, 1946, 610, 1731, 2484, 415, 2565, 2419, 2123, 1494, 1055, 2074, 2176, 1417, 1504, 1594, 2451, 2507, 772, 196, 442, 1226, 398, 253, 1276, 2491, 106, 2057, 2252, 370, 2386, 89, 802, 2467, 1672, 803, 796, 838, 880, 1228, 1613, 1310, 177, 812, 909, 198, 949, 1177, 2454, 2438, 1453, 64, 1144, 2039, 2239, 2233, 831, 1396, 2592, 742, 2098, 2076, 598, 1783, 74, 859, 1075, 1062, 1238, 713, 2504, 1415, 1089, 2461, 850, 1646, 1, 1308, 263, 1036, 935, 705, 1300, 971, 1368, 1662, 1826, 1524, 2548, 414, 1957, 2450, 331, 2250, 2200, 1134, 1637, 2489, 744, 1803, 1981, 947, 2615, 696, 267, 2092, 119, 2307, 2605, 622, 1887, 1848, 278, 1690, 2415, 687, 2158, 2542, 1324, 30, 2040, 2556, 1923, 2448, 100, 315, 2028, 1503, 2197, 1306, 452, 959, 1727, 440, 1192, 699, 2401, 245, 2166, 72, 1589, 1892, 1235, 1753, 27, 1522, 112, 939, 167, 482, 630, 2258, 2031, 1143, 2370, 1172, 33, 194, 958, 1912, 2337, 1287, 1697, 2417, 783, 1984, 1542, 593, 361, 2107, 816, 1518, 2446, 109, 792, 1219, 504, 747, 1925, 1224, 2397, 1257, 2030, 1046, 433, 1526, 391, 2152, 2481, 1270, 306, 672, 826, 1940, 2499, 1340, 686, 806, 2072, 1239, 2460, 1110, 2011, 758, 83, 1634, 2106, 1600, 1119, 810, 819, 2171, 562, 2399, 2173, 2440, 1664, 2062, 2135, 1304, 68, 2008, 1101, 856, 1378, 985, 1146, 170, 770, 781, 399, 2180, 2411, 1751, 1418, 215, 2212, 384, 1216, 412, 681, 1016, 132, 801, 25, 1688, 2347, 2080, 965, 1081, 2313, 348, 605, 2309, 1992, 1621, 1096, 2188, 2416, 647, 85, 2582, 2350, 2064, 1207, 1284, 2167, 2052, 503, 805, 329, 874, 19, 1836, 608, 367, 1874, 2102, 1400, 988, 2104, 1108, 1601, 446, 1739, 2513, 1250, 2257, 2001, 628, 289, 682, 1377, 257, 1271, 241, 2372, 723, 1852, 2413, 1092, 379, 1603, 837, 2029, 2146, 2190, 917, 70, 1149, 659, 1982, 2444, 1095, 1755, 944, 2380, 2148, 1771, 2143, 2118, 2019, 916, 123, 1251, 1939, 1898, 363, 2352, 2105, 80, 182, 1676, 1671, 408, 342, 93, 1422, 1195, 1651, 996, 2508, 1833, 2070, 1670, 597, 344, 1423, 2045, 2482, 695, 22, 823, 968, 1334, 454, 650, 2247, 1891, 1090, 82, 1915, 970, 1596, 1212, 706, 924, 1004, 789, 2082, 1021, 2433, 1301, 1523, 931, 1543, 2535, 1886, 1839, 2232, 980, 1252, 219, 1261, 2217, 975, 902, 1013, 78, 2305, 2236, 1330, 20, 761, 1097, 683, 684, 933, 2013, 1618, 2312, 910, 2065, 1080, 2283, 2363, 321, 2069, 848, 953, 1605, 992, 62, 1588, 1903, 1572, 704, 410, 918, 1822, 2345, 760, 2093, 1936, 925, 383, 844, 1246, 938, 2157, 2201, 2270, 2452, 366, 359, 1302, 2303, 1385, 1513, 660, 434, 1880, 1743, 1326, 2117, 2308, 1569, 18, 2617, 710, 1031, 2455, 2465, 2329, 313, 411, 2496, 2003, 1050, 1548, 1698, 1775, 793, 213, 274, 865, 1296, 2483, 1837, 964, 499, 1845, 1767, 626, 2443, 1030, 1888, 2389, 556, 2192, 1558, 1566, 90, 728, 998, 658, 1165, 841, 1996, 1311, 1761, 2540, 750, 1868, 1184, 832, 1645, 651, 1824, 1345, 2524, 2384, 774, 247, 2168, 1875, 1035, 77, 1533, 864, 486, 1729, 2522, 2269, 1514, 24, 1240, 1877, 1100, 1528, 1001, 1113, 1158, 1099, 617, 256, 2511, 1811, 423, 1181, 1188, 407, 1087, 2261, 993, 2478, 2012, 1606, 1074, 2395, 448, 1995, 1735, 76, 1865, 325, 1636, 1520, 483, 807, 358, 945, 1693, 1218, 2325, 895, 2091, 1174, 479, 2464, 623, 548, 2383, 1901, 2310, 2607, 1797, 2520, 811, 2246, 2044, 882, 2346, 2332, 1587, 1341, 2449, 189, 2179, 66, 714, 1555, 907, 1241, 1616, 842, 456, 2170, 829, 2036, 91, 388, 635, 362, 1825, 2425, 2164, 1680, 847, 1353, 2407, 633, 86, 2294, 2094, 2534, 751, 616, 1026, 922, 671, 2293, 73, 2568, 1997, 804, 2518, 1937, 1292, 1501, 2090, 1180, 2286, 1285, 71, 1827, 1899, 1553, 2447, 920, 2219, 1000, 2109, 1293, 2000, 1955, 730, 759, 835, 1155, 1109, 1889, 1916, 2077, 2047, 2614, 1829, 991, 1737, 2210, 912, 1349, 1037, 102, 2624, 1612, 1681, 1443, 246, 2623, 288, 2218, 1289, 936, 2267, 994, 435, 340, 2078, 732, 919, 2400, 2306, 1630, 1178, 2204, 1200, 1947, 1124, 1086, 489, 2237, 333, 1551, 1386, 317, 2445, 1408, 1404, 2342, 952, 32, 1560, 2599, 2559, 2142, 2376, 1763, 1879, 1932, 934, 1273, 2178, 1648, 281, 1012, 817, 2512, 1507, 2273, 930, 898, 1628, 88, 318, 1883, 2264, 1759, 2567, 2586, 896, 1517, 2536, 755, 1048, 5, 427, 357, 2259, 2324, 1538, 1067, 2391, 1702, 2430, 2177, 2302, 700, 782, 2349, 1364, 2221, 878, 1695, 111, 2374, 591, 1194, 323, 2490, 1632, 343, 507, 432, 185, 1917, 887, 927, 2032, 1266, 63, 1549, 1394, 1104, 120, 451, 1259, 977, 1027, 1152, 676, 1991, 1709, 385, 1717, 1342, 2193, 1831, 2009, 685, 2134, 1855, 1213, 818, 1627, 1111, 1851, 421, 891, 1617, 1358, 775, 352, 1186, 614, 2046, 2087, 1033, 908, 861, 1529, 1579, 188, 251, 67, 2500, 1791, 2393, 115, 711, 1931, 2130, 99, 1022, 2129, 2601, 609, 2163, 1307, 2526, 1972, 1372, 1919, 893, 1983, 1116, 1024, 2083, 441, 505, 193, 1571, 2527, 1834, 1696, 1913, 2550, 2463, 2330, 1987, 693, 1254, 2371, 1789, 2627, 1611, 1221, 390, 1283, 105, 940, 1910, 389, 1506, 943, 1049, 2541, 510, 607, 2271, 1843, 1248, 724, 360, 1563, 1020, 619, 2245, 2144, 186, 496, 2470, 828, 416, 2005, 1069, 255, 130, 2343, 2299, 2187, 1060, 2594, 1427, 1575, 2502, 187, 2458, 673, 2516, 777, 1371, 1675, 450, 1496, 821, 2114, 1197, 1773, 612, 2297, 2473, 2424, 2215, 2621, 718, 1297, 1231, 1546, 1838, 1850, 1265, 2055, 2043, 2600, 1719, 2378, 1424, 1935, 615, 1583, 2514, 394, 1267, 784, 2235, 2602, 2132, 1980, 1374, 1045, 901, 1203, 640, 655, 1085, 1557, 498, 1291, 1968, 217, 387, 1040, 1367, 1561, 855, 667, 2360, 1591, 2523, 1635, 1393, 221, 2576, 2379, 906, 1873, 2124, 400, 1682, 1999, 1581, 2537, 244, 2382, 2531, 1023, 2223, 738, 1103, 599, 404, 1202, 2505, 168, 1282, 1061, 1871, 1608, 1397, 417, 214, 2613, 1577, 2095, 2356, 1989, 2547, 2589, 1928, 1498, 1347, 2208, 915, 1857, 621, 2339, 276, 1237, 1993, 1198, 6, 350, 2068, 1638, 26, 1499, 740, 1363, 1047, 2127, 107, 2006, 2160, 995, 2427, 2020, 1032, 2139, 11, 680, 637, 2543, 1002, 2113, 2016, 1137, 2198, 1077, 356, 2422, 1660, 131, 95, 1343, 776, 1592, 788, 1449, 888, 2101, 2249, 990, 1166, 1856, 2426, 1008, 2588, 2572, 1072, 1387, 2053, 2551, 2519, 839, 1862, 729, 1355, 1885, 1556, 701, 172, 679, 1704, 1585, 1666, 2373, 1389, 2580, 87, 2131, 1537, 2517, 1619, 1333, 1869, 773, 2181, 731, 1163, 1590, 319, 2336, 1654, 1678, 419, 987, 1091, 1384, 418, 2533, 1933, 1105, 402, 646, 963, 1314, 2619, 9, 1905, 604, 176, 643, 594, 1849, 1280, 351, 381, 1398, 1911, 2466, 553, 950, 2316, 766, 1626, 1950, 309, 2125, 328, 2122, 1893, 2574, 1354, 2538, 1320, 2472, 978, 1663, 101, 1179, 2128, 1694, 401, 2475, 984, 1391, 956, 2575, 746, 2209, 1614, 2056, 2398, 1864, 1362, 1990, 2488, 1227, 2, 1299, 1745, 1929, 2510, 2364, 1741, 2260, 307, 128, 725, 815, 631, 2017, 1813, 2318, 1959, 500, 1711, 2287, 2242, 2584, 1667, 2532, 670, 447, 1544, 2279, 2111, 2136, 2024, 2085, 560, 1106, 663, 2611, 2015, 765, 1357, 429, 1552, 14, 2301, 2314, 851, 757, 2344, 1168, 2569, 691, 1949, 1204, 1967, 899, 885, 1011, 420, 1777, 2494, 969, 2375, 97, 61, 1904, 2174, 688, 1360, 2353, 1747, 438, 2528, 1881, 1366],
    _phantom: core::marker::PhantomData,
};

pub(super) static KEYSYM_TO_UPPER_KEYSYM: PhfMap<u32, KeysymCaseMapping> = PhfMap {
    key: 12913932095322966823,
    disps: &[(0, 20), (0, 9), (0, 0), (0, 0), (0, 2), (0, 10), (0, 6), (0, 0), (0, 9), (0, 1), (0, 22), (0, 15), (0, 0), (0, 7), (0, 0), (0, 13), (0, 3), (0, 0), (0, 45), (0, 1), (0, 1), (0, 0), (0, 17), (0, 35), (0, 3), (0, 0), (0, 104), (0, 76), (0, 0), (0, 12), (0, 0), (0, 83), (0, 44), (0, 78), (0, 13), (0, 115), (0, 0), (0, 82), (0, 4), (0, 1), (0, 1), (0, 150), (0, 41), (0, 2), (0, 1), (0, 61), (0, 97), (0, 13), (1, 109), (0, 1), (0, 73), (0, 0), (0, 43), (0, 32), (0, 1), (0, 36), (0, 3), (0, 4), (0, 35), (0, 0), (0, 111), (0, 123), (0, 0), (0, 0)],
    map: &[KeysymCaseMapping { keysym: 0x06d7, other: 0x000006f7 }, KeysymCaseMapping { keysym: 0x07e9, other: 0x000007c9 }, KeysymCaseMapping { keysym: 0x06aa, other: 0x000006ba }, KeysymCaseMapping { keysym: 0x00fe, other: 0x000000de }, KeysymCaseMapping { keysym: 0x00ed, other: 0x000000cd }, KeysymCaseMapping { keysym: 0x03f3, other: 0x000003d3 }, KeysymCaseMapping { keysym: 0x00f9, other: 0x000000d9 }, KeysymCaseMapping { keysym: 0x06c1, other: 0x000006e1 }, KeysymCaseMapping { keysym: 0x02b1, other: 0x000002a1 }, KeysymCaseMapping { keysym: 0x07f7, other: 0x000007d7 }, KeysymCaseMapping { keysym: 0x06c6, other: 0x000006e6 }, KeysymCaseMapping { keysym: 0x00f3, other: 0x000000d3 }, KeysymCaseMapping { keysym: 0x00fb, other: 0x000000db }, KeysymCaseMapping { keysym: 0x07f0, other: 0x000007d0 }, KeysymCaseMapping { keysym: 0x00e0, other: 0x000000c0 }, KeysymCaseMapping { keysym: 0x03e7, other: 0x000003c7 }, KeysymCaseMapping { keysym: 0x07f3, other: 0x000007d2 }, KeysymCaseMapping { keysym: 0x00ec, other: 0x000000cc }, KeysymCaseMapping { keysym: 0x00e8, other: 0x000000c8 }, KeysymCaseMapping { keysym: 0x00ea, other: 0x000000ca }, KeysymCaseMapping { keysym: 0x02f5, other: 0x000002d5 }, KeysymCaseMapping { keysym: 0x07ef, other: 0x000007cf }, KeysymCaseMapping { keysym: 0x01ea, other: 0x000001ca }, KeysymCaseMapping { keysym: 0x00ef, other: 0x000000cf }, KeysymCaseMapping { keysym: 0x06d4, other: 0x000006f4 }, KeysymCaseMapping { keysym: 0x02bb, other: 0x000002ab }, KeysymCaseMapping { keysym: 0x06c3, other: 0x000006e3 }, KeysymCaseMapping { keysym: 0x01ba, other: 0x000001aa }, KeysymCaseMapping { keysym: 0x00f0, other: 0x000000d0 }, KeysymCaseMapping { keysym: 0x03e0, other: 0x000003c0 }, KeysymCaseMapping { keysym: 0x06af, other: 0x000006bf }, KeysymCaseMapping { keysym: 0x00fd, other: 0x000000dd }, KeysymCaseMapping { keysym: 0x06d2, other: 0x000006f2 }, KeysymCaseMapping { keysym: 0x00f2, other: 0x000000d2 }, KeysymCaseMapping { keysym: 0x00fa, other: 0x000000da }, KeysymCaseMapping { keysym: 0x06d6, other: 0x000006f6 }, KeysymCaseMapping { keysym: 0x00df, other: 0x01001e9e }, KeysymCaseMapping { keysym: 0x02fd, other: 0x000002dd }, KeysymCaseMapping { keysym: 0x07e1, other: 0x000007c1 }, KeysymCaseMapping { keysym: 0x06a5, other: 0x000006b5 }, KeysymCaseMapping { keysym: 0x00e1, other: 0x000000c1 }, KeysymCaseMapping { keysym: 0x07f2, other: 0x000007d2 }, KeysymCaseMapping { keysym: 0x01b5, other: 0x000001a5 }, KeysymCaseMapping { keysym: 0x07ed, other: 0x000007cd }, KeysymCaseMapping { keysym: 0x06d9, other: 0x000006f9 }, KeysymCaseMapping { keysym: 0x01bc, other: 0x000001ac }, KeysymCaseMapping { keysym: 0x07b8, other: 0x000007a8 }, KeysymCaseMapping { keysym: 0x06dc, other: 0x000006fc }, KeysymCaseMapping { keysym: 0x07e4, other: 0x000007c4 }, KeysymCaseMapping { keysym: 0x06ac, other: 0x000006bc }, KeysymCaseMapping { keysym: 0x07e2, other: 0x000007c2 }, KeysymCaseMapping { keysym: 0x00f4, other: 0x000000d4 }, KeysymCaseMapping { keysym: 0x06c8, other: 0x000006e8 }, KeysymCaseMapping { keysym: 0x03b6, other: 0x000003a6 }, KeysymCaseMapping { keysym: 0x03bf, other: 0x000003bd }, KeysymCaseMapping { keysym: 0x07f1, other: 0x000007d1 }, KeysymCaseMapping { keysym: 0x00ee, other: 0x000000ce }, KeysymCaseMapping { keysym: 0x07e3, other: 0x000007c3 }, KeysymCaseMapping { keysym: 0x00e3, other: 0x000000c3 }, KeysymCaseMapping { keysym: 0x01bf, other: 0x000001af }, KeysymCaseMapping { keysym: 0x02b6, other: 0x000002a6 }, KeysymCaseMapping { keysym: 0x02f8, other: 0x000002d8 }, KeysymCaseMapping { keysym: 0x07f4, other: 0x000007d4 }, KeysymCaseMapping { keysym: 0x01be, other: 0x000001ae }, KeysymCaseMapping { keysym: 0x01b9, other: 0x000001a9 }, KeysymCaseMapping { keysym: 0x06c7, other: 0x000006e7 }, KeysymCaseMapping { keysym: 0x02b9, other: 0x00000049 }, KeysymCaseMapping { keysym: 0x07f6, other: 0x000007d6 }, KeysymCaseMapping { keysym: 0x00f1, other: 0x000000d1 }, KeysymCaseMapping { keysym: 0x06ca, other: 0x000006ea }, KeysymCaseMapping { keysym: 0x00f6, other: 0x000000d6 }, KeysymCaseMapping { keysym: 0x01f1, other: 0x000001d1 }, KeysymCaseMapping { keysym: 0x06db, other: 0x000006fb }, KeysymCaseMapping { keysym: 0x13bd, other: 0x000013bc }, KeysymCaseMapping { keysym: 0x07e5, other: 0x000007c5 }, KeysymCaseMapping { keysym: 0x07b3, other: 0x000007a3 }, KeysymCaseMapping { keysym: 0x06d0, other: 0x000006f0 }, KeysymCaseMapping { keysym: 0x06d8, other: 0x000006f8 }, KeysymCaseMapping { keysym: 0x01e0, other: 0x000001c0 }, KeysymCaseMapping { keysym: 0x06cd, other: 0x000006ed }, KeysymCaseMapping { keysym: 0x03b3, other: 0x000003a3 }, KeysymCaseMapping { keysym: 0x01b6, other: 0x000001a6 }, KeysymCaseMapping { keysym: 0x06c9, other: 0x000006e9 }, KeysymCaseMapping { keysym: 0x01fe, other: 0x000001de }, KeysymCaseMapping { keysym: 0x07b5, other: 0x000007a5 }, KeysymCaseMapping { keysym: 0x07e8, other: 0x000007c8 }, KeysymCaseMapping { keysym: 0x03b5, other: 0x000003a5 }, KeysymCaseMapping { keysym: 0x03f9, other: 0x000003d9 }, KeysymCaseMapping { keysym: 0x06cc, other: 0x000006ec }, KeysymCaseMapping { keysym: 0x02e6, other: 0x000002c6 }, KeysymCaseMapping { keysym: 0x01fb, other: 0x000001db }, KeysymCaseMapping { keysym: 0x01bb, other: 0x000001ab }, KeysymCaseMapping { keysym: 0x07e7, other: 0x000007c7 }, KeysymCaseMapping { keysym: 0x06dd, other: 0x000006fd }, KeysymCaseMapping { keysym: 0x00e7, other: 0x000000c7 }, KeysymCaseMapping { keysym: 0x00e2, other: 0x000000c2 }, KeysymCaseMapping { keysym: 0x06da, other: 0x000006fa }, KeysymCaseMapping { keysym: 0x06a4, other: 0x000006b4 }, KeysymCaseMapping { keysym: 0x06df, other: 0x000006ff }, KeysymCaseMapping { keysym: 0x06a1, other: 0x000006b1 }, KeysymCaseMapping { keysym: 0x07f9, other: 0x000007d9 }, KeysymCaseMapping { keysym: 0x01f0, other: 0x000001d0 }, KeysymCaseMapping { keysym: 0x07f5, other: 0x000007d5 }, KeysymCaseMapping { keysym: 0x06a6, other: 0x000006b6 }, KeysymCaseMapping { keysym: 0x07ea, other: 0x000007ca }, KeysymCaseMapping { keysym: 0x06ce, other: 0x000006ee }, KeysymCaseMapping { keysym: 0x01f5, other: 0x000001d5 }, KeysymCaseMapping { keysym: 0x03fe, other: 0x000003de }, KeysymCaseMapping { keysym: 0x01ef, other: 0x000001cf }, KeysymCaseMapping { keysym: 0x06ae, other: 0x000006be }, KeysymCaseMapping { keysym: 0x06a2, other: 0x000006b2 }, KeysymCaseMapping { keysym: 0x07b7, other: 0x000007a7 }, KeysymCaseMapping { keysym: 0x00e4, other: 0x000000c4 }, KeysymCaseMapping { keysym: 0x07b4, other: 0x000007a4 }, KeysymCaseMapping { keysym: 0x00f8, other: 0x000000d8 }, KeysymCaseMapping { keysym: 0x01f8, other: 0x000001d8 }, KeysymCaseMapping { keysym: 0x06d1, other: 0x000006f1 }, KeysymCaseMapping { keysym: 0x06a9, other: 0x000006b9 }, KeysymCaseMapping { keysym: 0x03bb, other: 0x000003ab }, KeysymCaseMapping { keysym: 0x06a3, other: 0x000006b3 }, KeysymCaseMapping { keysym: 0x01f2, other: 0x000001d2 }, KeysymCaseMapping { keysym: 0x06cb, other: 0x000006eb }, KeysymCaseMapping { keysym: 0x03fd, other: 0x000003dd }, KeysymCaseMapping { keysym: 0x06c0, other: 0x000006e0 }, KeysymCaseMapping { keysym: 0x03f2, other: 0x000003d2 }, KeysymCaseMapping { keysym: 0x03ef, other: 0x000003cf }, KeysymCaseMapping { keysym: 0x07b1, other: 0x000007a1 }, KeysymCaseMapping { keysym: 0x03ec, other: 0x000003cc }, KeysymCaseMapping { keysym: 0x03ba, other: 0x000003aa }, KeysymCaseMapping { keysym: 0x07e6, other: 0x000007c6 }, KeysymCaseMapping { keysym: 0x07b2, other: 0x000007a2 }, KeysymCaseMapping { keysym: 0x00e6, other: 0x000000c6 }, KeysymCaseMapping { keysym: 0x01b1, other: 0x000001a1 }, KeysymCaseMapping { keysym: 0x06de, other: 0x000006fe }, KeysymCaseMapping { keysym: 0x02bc, other: 0x000002ac }, KeysymCaseMapping { keysym: 0x06d3, other: 0x000006f3 }, KeysymCaseMapping { keysym: 0x01e6, other: 0x000001c6 }, KeysymCaseMapping { keysym: 0x00f5, other: 0x000000d5 }, KeysymCaseMapping { keysym: 0x07ec, other: 0x000007cc }, KeysymCaseMapping { keysym: 0x07ee, other: 0x000007ce }, KeysymCaseMapping { keysym: 0x00e9, other: 0x000000c9 }, KeysymCaseMapping { keysym: 0x06c2, other: 0x000006e2 }, KeysymCaseMapping { keysym: 0x07b9, other: 0x000007a9 }, KeysymCaseMapping { keysym: 0x07f8, other: 0x000007d8 }, KeysymCaseMapping { keysym: 0x01e3, other: 0x000001c3 }, KeysymCaseMapping { keysym: 0x06ab, other: 0x000006bb }, KeysymCaseMapping { keysym: 0x02fe, other: 0x000002de }, KeysymCaseMapping { keysym: 0x06a8, other: 0x000006b8 }, KeysymCaseMapping { keysym: 0x03bc, other: 0x000003ac }, KeysymCaseMapping { keysym: 0x00fc, other: 0x000000dc }, KeysymCaseMapping { keysym: 0x06ad, other: 0x000006bd }, KeysymCaseMapping { keysym: 0x01b3, other: 0x000001a3 }, KeysymCaseMapping { keysym: 0x00ff, other: 0x000013be }, KeysymCaseMapping { keysym: 0x01e8, other: 0x000001c8 }, KeysymCaseMapping { keysym: 0x06a7, other: 0x000006b7 }, KeysymCaseMapping { keysym: 0x06c5, other: 0x000006e5 }, KeysymCaseMapping { keysym: 0x03f1, other: 0x000003d1 }, KeysymCaseMapping { keysym: 0x00eb, other: 0x000000cb }, KeysymCaseMapping { keysym: 0x00e5, other: 0x000000c5 }, KeysymCaseMapping { keysym: 0x06c4, other: 0x000006e4 }, KeysymCaseMapping { keysym: 0x06cf, other: 0x000006ef }, KeysymCaseMapping { keysym: 0x07bb, other: 0x000007ab }, KeysymCaseMapping { keysym: 0x01ec, other: 0x000001cc }, KeysymCaseMapping { keysym: 0x07eb, other: 0x000007cb }, KeysymCaseMapping { keysym: 0x02e5, other: 0x000002c5 }, KeysymCaseMapping { keysym: 0x01e5, other: 0x000001c5 }, KeysymCaseMapping { keysym: 0x00b5, other: 0x000007cc }, KeysymCaseMapping { keysym: 0x01f9, other: 0x000001d9 }, KeysymCaseMapping { keysym: 0x08f6, other: 0x01000191 }, KeysymCaseMapping { keysym: 0x06d5, other: 0x000006f5 }],
    _phantom: core::marker::PhantomData,
};

pub(super) static KEYSYM_TO_LOWER_KEYSYM: PhfMap<u32, KeysymCaseMapping> = PhfMap {
    key: 12913932095322966823,
    disps: &[(0, 1), (0, 2), (0, 0), (0, 3), (0, 21), (0, 15), (0, 8), (0, 3), (0, 0), (0, 3), (0, 3), (0, 52), (0, 76), (0, 8), (0, 16), (0, 5), (0, 0), (0, 17), (0, 0), (0, 0), (0, 8), (0, 0), (0, 0), (0, 25), (0, 71), (0, 24), (0, 3), (0, 10), (0, 4), (0, 4), (0, 2), (0, 76), (0, 0), (0, 99), (0, 1), (0, 15), (0, 37), (0, 4), (0, 6), (0, 53), (0, 0), (0, 0), (0, 5), (0, 8), (0, 37), (0, 0), (0, 48), (0, 23), (0, 6), (0, 30), (0, 3), (0, 8), (0, 42), (0, 24), (1, 16), (0, 0), (1, 62), (1, 61), (0, 17), (0, 0), (0, 66), (0, 0), (0, 2), (0, 26)],
    map: &[KeysymCaseMapping { keysym: 0x06b5, other: 0x000006a5 }, KeysymCaseMapping { keysym: 0x07d9, other: 0x000007f9 }, KeysymCaseMapping { keysym: 0x00da, other: 0x000000fa }, KeysymCaseMapping { keysym: 0x01de, other: 0x000001fe }, KeysymCaseMapping { keysym: 0x03d9, other: 0x000003f9 }, KeysymCaseMapping { keysym: 0x03ac, other: 0x000003bc }, KeysymCaseMapping { keysym: 0x06e8, other: 0x000006c8 }, KeysymCaseMapping { keysym: 0x00d2, other: 0x000000f2 }, KeysymCaseMapping { keysym: 0x01c5, other: 0x000001e5 }, KeysymCaseMapping { keysym: 0x07a8, other: 0x000007b8 }, KeysymCaseMapping { keysym: 0x07ca, other: 0x000007ea }, KeysymCaseMapping { keysym: 0x06e4, other: 0x000006c4 }, KeysymCaseMapping { keysym: 0x07d5, other: 0x000007f5 }, KeysymCaseMapping { keysym: 0x06e9, other: 0x000006c9 }, KeysymCaseMapping { keysym: 0x07a3, other: 0x000007b3 }, KeysymCaseMapping { keysym: 0x01ca, other: 0x000001ea }, KeysymCaseMapping { keysym: 0x06f9, other: 0x000006d9 }, KeysymCaseMapping { keysym: 0x06fe, other: 0x000006de }, KeysymCaseMapping { keysym: 0x01d5, other: 0x000001f5 }, KeysymCaseMapping { keysym: 0x01a3, other: 0x000001b3 }, KeysymCaseMapping { keysym: 0x00c3, other: 0x000000e3 }, KeysymCaseMapping { keysym: 0x00cd, other: 0x000000ed }, KeysymCaseMapping { keysym: 0x00c7, other: 0x000000e7 }, KeysymCaseMapping { keysym: 0x00c6, other: 0x000000e6 }, KeysymCaseMapping { keysym: 0x00dd, other: 0x000000fd }, KeysymCaseMapping { keysym: 0x00c8, other: 0x000000e8 }, KeysymCaseMapping { keysym: 0x03bd, other: 0x000003bf }, KeysymCaseMapping { keysym: 0x07d0, other: 0x000007f0 }, KeysymCaseMapping { keysym: 0x06bb, other: 0x000006ab }, KeysymCaseMapping { keysym: 0x02a6, other: 0x000002b6 }, KeysymCaseMapping { keysym: 0x02dd, other: 0x000002fd }, KeysymCaseMapping { keysym: 0x06ef, other: 0x000006cf }, KeysymCaseMapping { keysym: 0x07cb, other: 0x000007eb }, KeysymCaseMapping { keysym: 0x03a3, other: 0x000003b3 }, KeysymCaseMapping { keysym: 0x06ee, other: 0x000006ce }, KeysymCaseMapping { keysym: 0x03c0, other: 0x000003e0 }, KeysymCaseMapping { keysym: 0x01c6, other: 0x000001e6 }, KeysymCaseMapping { keysym: 0x01d0, other: 0x000001f0 }, KeysymCaseMapping { keysym: 0x00d3, other: 0x000000f3 }, KeysymCaseMapping { keysym: 0x06f4, other: 0x000006d4 }, KeysymCaseMapping { keysym: 0x02ab, other: 0x000002bb }, KeysymCaseMapping { keysym: 0x07d6, other: 0x000007f6 }, KeysymCaseMapping { keysym: 0x06b1, other: 0x000006a1 }, KeysymCaseMapping { keysym: 0x01c0, other: 0x000001e0 }, KeysymCaseMapping { keysym: 0x06e5, other: 0x000006c5 }, KeysymCaseMapping { keysym: 0x01ae, other: 0x000001be }, KeysymCaseMapping { keysym: 0x06b6, other: 0x000006a6 }, KeysymCaseMapping { keysym: 0x13be, other: 0x000000ff }, KeysymCaseMapping { keysym: 0x02a1, other: 0x000002b1 }, KeysymCaseMapping { keysym: 0x02d8, other: 0x000002f8 }, KeysymCaseMapping { keysym: 0x06ff, other: 0x000006df }, KeysymCaseMapping { keysym: 0x01db, other: 0x000001fb }, KeysymCaseMapping { keysym: 0x00c9, other: 0x000000e9 }, KeysymCaseMapping { keysym: 0x07cc, other: 0x000007ec }, KeysymCaseMapping { keysym: 0x07c7, other: 0x000007e7 }, KeysymCaseMapping { keysym: 0x07c1, other: 0x000007e1 }, KeysymCaseMapping { keysym: 0x01a9, other: 0x000001b9 }, KeysymCaseMapping { keysym: 0x06e0, other: 0x000006c0 }, KeysymCaseMapping { keysym: 0x07d1, other: 0x000007f1 }, KeysymCaseMapping { keysym: 0x00de, other: 0x000000fe }, KeysymCaseMapping { keysym: 0x06bc, other: 0x000006ac }, KeysymCaseMapping { keysym: 0x06b7, other: 0x000006a7 }, KeysymCaseMapping { keysym: 0x06f5, other: 0x000006d5 }, KeysymCaseMapping { keysym: 0x02de, other: 0x000002fe }, KeysymCaseMapping { keysym: 0x01d1, other: 0x000001f1 }, KeysymCaseMapping { keysym: 0x00d4, other: 0x000000f4 }, KeysymCaseMapping { keysym: 0x01cc, other: 0x000001ec }, KeysymCaseMapping { keysym: 0x07a4, other: 0x000007b4 }, KeysymCaseMapping { keysym: 0x01af, other: 0x000001bf }, KeysymCaseMapping { keysym: 0x00c2, other: 0x000000e2 }, KeysymCaseMapping { keysym: 0x06eb, other: 0x000006cb }, KeysymCaseMapping { keysym: 0x06e6, other: 0x000006c6 }, KeysymCaseMapping { keysym: 0x07d7, other: 0x000007f7 }, KeysymCaseMapping { keysym: 0x00d9, other: 0x000000f9 }, KeysymCaseMapping { keysym: 0x00cf, other: 0x000000ef }, KeysymCaseMapping { keysym: 0x06f0, other: 0x000006d0 }, KeysymCaseMapping { keysym: 0x02ac, other: 0x000002bc }, KeysymCaseMapping { keysym: 0x00c4, other: 0x000000e4 }, KeysymCaseMapping { keysym: 0x07a5, other: 0x000007b5 }, KeysymCaseMapping { keysym: 0x07c6, other: 0x000007e6 }, KeysymCaseMapping { keysym: 0x03cc, other: 0x000003ec }, KeysymCaseMapping { keysym: 0x01aa, other: 0x000001ba }, KeysymCaseMapping { keysym: 0x01a5, other: 0x000001b5 }, KeysymCaseMapping { keysym: 0x00c5, other: 0x000000e5 }, KeysymCaseMapping { keysym: 0x07ab, other: 0x000007bb }, KeysymCaseMapping { keysym: 0x00d8, other: 0x000000f8 }, KeysymCaseMapping { keysym: 0x07cd, other: 0x000007ed }, KeysymCaseMapping { keysym: 0x13bc, other: 0x000013bd }, KeysymCaseMapping { keysym: 0x06fb, other: 0x000006db }, KeysymCaseMapping { keysym: 0x00ca, other: 0x000000ea }, KeysymCaseMapping { keysym: 0x06bd, other: 0x000006ad }, KeysymCaseMapping { keysym: 0x01cf, other: 0x000001ef }, KeysymCaseMapping { keysym: 0x07c2, other: 0x000007e2 }, KeysymCaseMapping { keysym: 0x07d2, other: 0x000007f2 }, KeysymCaseMapping { keysym: 0x06f8, other: 0x000006d8 }, KeysymCaseMapping { keysym: 0x06e1, other: 0x000006c1 }, KeysymCaseMapping { keysym: 0x03a5, other: 0x000003b5 }, KeysymCaseMapping { keysym: 0x01d2, other: 0x000001f2 }, KeysymCaseMapping { keysym: 0x00d5, other: 0x000000f5 }, KeysymCaseMapping { keysym: 0x07c8, other: 0x000007e8 }, KeysymCaseMapping { keysym: 0x00c0, other: 0x000000e0 }, KeysymCaseMapping { keysym: 0x06b8, other: 0x000006a8 }, KeysymCaseMapping { keysym: 0x06b3, other: 0x000006a3 }, KeysymCaseMapping { keysym: 0x06ec, other: 0x000006cc }, KeysymCaseMapping { keysym: 0x06f6, other: 0x000006d6 }, KeysymCaseMapping { keysym: 0x03d2, other: 0x000003f2 }, KeysymCaseMapping { keysym: 0x06f1, other: 0x000006d1 }, KeysymCaseMapping { keysym: 0x06fa, other: 0x000006da }, KeysymCaseMapping { keysym: 0x07c3, other: 0x000007e3 }, KeysymCaseMapping { keysym: 0x01a6, other: 0x000001b6 }, KeysymCaseMapping { keysym: 0x03aa, other: 0x000003ba }, KeysymCaseMapping { keysym: 0x00d0, other: 0x000000f0 }, KeysymCaseMapping { keysym: 0x01ab, other: 0x000001bb }, KeysymCaseMapping { keysym: 0x03d1, other: 0x000003f1 }, KeysymCaseMapping { keysym: 0x06ea, other: 0x000006ca }, KeysymCaseMapping { keysym: 0x06be, other: 0x000006ae }, KeysymCaseMapping { keysym: 0x01c8, other: 0x000001e8 }, KeysymCaseMapping { keysym: 0x07ce, other: 0x000007ee }, KeysymCaseMapping { keysym: 0x07a9, other: 0x000007b9 }, KeysymCaseMapping { keysym: 0x06f7, other: 0x000006d7 }, KeysymCaseMapping { keysym: 0x03c7, other: 0x000003e7 }, KeysymCaseMapping { keysym: 0x03a6, other: 0x000003b6 }, KeysymCaseMapping { keysym: 0x02c5, other: 0x000002e5 }, KeysymCaseMapping { keysym: 0x01d8, other: 0x000001f8 }, KeysymCaseMapping { keysym: 0x07a1, other: 0x000007b1 }, KeysymCaseMapping { keysym: 0x01a1, other: 0x000001b1 }, KeysymCaseMapping { keysym: 0x00c1, other: 0x000000e1 }, KeysymCaseMapping { keysym: 0x07d8, other: 0x000007f8 }, KeysymCaseMapping { keysym: 0x06b2, other: 0x000006a2 }, KeysymCaseMapping { keysym: 0x00cb, other: 0x000000eb }, KeysymCaseMapping { keysym: 0x00db, other: 0x000000fb }, KeysymCaseMapping { keysym: 0x00d6, other: 0x000000f6 }, KeysymCaseMapping { keysym: 0x07c9, other: 0x000007e9 }, KeysymCaseMapping { keysym: 0x02c6, other: 0x000002e6 }, KeysymCaseMapping { keysym: 0x07c4, other: 0x000007e4 }, KeysymCaseMapping { keysym: 0x06b4, other: 0x000006a4 }, KeysymCaseMapping { keysym: 0x07a7, other: 0x000007b7 }, KeysymCaseMapping { keysym: 0x02a9, other: 0x00000069 }, KeysymCaseMapping { keysym: 0x06f2, other: 0x000006d2 }, KeysymCaseMapping { keysym: 0x03dd, other: 0x000003fd }, KeysymCaseMapping { keysym: 0x00ce, other: 0x000000ee }, KeysymCaseMapping { keysym: 0x03ab, other: 0x000003bb }, KeysymCaseMapping { keysym: 0x06b9, other: 0x000006a9 }, KeysymCaseMapping { keysym: 0x06e2, other: 0x000006c2 }, KeysymCaseMapping { keysym: 0x01ac, other: 0x000001bc }, KeysymCaseMapping { keysym: 0x06fc, other: 0x000006dc }, KeysymCaseMapping { keysym: 0x07a2, other: 0x000007b2 }, KeysymCaseMapping { keysym: 0x02d5, other: 0x000002f5 }, KeysymCaseMapping { keysym: 0x06bf, other: 0x000006af }, KeysymCaseMapping { keysym: 0x00d1, other: 0x000000f1 }, KeysymCaseMapping { keysym: 0x00cc, other: 0x000000ec }, KeysymCaseMapping { keysym: 0x06fd, other: 0x000006dd }, KeysymCaseMapping { keysym: 0x03d3, other: 0x000003f3 }, KeysymCaseMapping { keysym: 0x07d4, other: 0x000007f4 }, KeysymCaseMapping { keysym: 0x01c3, other: 0x000001e3 }, KeysymCaseMapping { keysym: 0x03de, other: 0x000003fe }, KeysymCaseMapping { keysym: 0x01d9, other: 0x000001f9 }, KeysymCaseMapping { keysym: 0x06e7, other: 0x000006c7 }, KeysymCaseMapping { keysym: 0x06e3, other: 0x000006c3 }, KeysymCaseMapping { keysym: 0x00dc, other: 0x000000fc }, KeysymCaseMapping { keysym: 0x07c5, other: 0x000007e5 }, KeysymCaseMapping { keysym: 0x06ed, other: 0x000006cd }, KeysymCaseMapping { keysym: 0x06ba, other: 0x000006aa }, KeysymCaseMapping { keysym: 0x07cf, other: 0x000007ef }, KeysymCaseMapping { keysym: 0x03cf, other: 0x000003ef }, KeysymCaseMapping { keysym: 0x06f3, other: 0x000006d3 }],
    _phantom: core::marker::PhantomData,
};

pub(super) static KEYSYM_TO_CHAR: &[KeysymChar] = &[
    KeysymChar {
        keysym: 0x0020,
        char: ' ',
    },
    KeysymChar {
        keysym: 0x0021,
        char: '!',
    },
    KeysymChar {
        keysym: 0x0022,
        char: '"',
    },
    KeysymChar {
        keysym: 0x0023,
        char: '#',
    },
    KeysymChar {
        keysym: 0x0024,
        char: '$',
    },
    KeysymChar {
        keysym: 0x0025,
        char: '%',
    },
    KeysymChar {
        keysym: 0x0026,
        char: '&',
    },
    KeysymChar {
        keysym: 0x0027,
        char: '\'',
    },
    KeysymChar {
        keysym: 0x0028,
        char: '(',
    },
    KeysymChar {
        keysym: 0x0029,
        char: ')',
    },
    KeysymChar {
        keysym: 0x002a,
        char: '*',
    },
    KeysymChar {
        keysym: 0x002b,
        char: '+',
    },
    KeysymChar {
        keysym: 0x002c,
        char: ',',
    },
    KeysymChar {
        keysym: 0x002d,
        char: '-',
    },
    KeysymChar {
        keysym: 0x002e,
        char: '.',
    },
    KeysymChar {
        keysym: 0x002f,
        char: '/',
    },
    KeysymChar {
        keysym: 0x0030,
        char: '0',
    },
    KeysymChar {
        keysym: 0x0031,
        char: '1',
    },
    KeysymChar {
        keysym: 0x0032,
        char: '2',
    },
    KeysymChar {
        keysym: 0x0033,
        char: '3',
    },
    KeysymChar {
        keysym: 0x0034,
        char: '4',
    },
    KeysymChar {
        keysym: 0x0035,
        char: '5',
    },
    KeysymChar {
        keysym: 0x0036,
        char: '6',
    },
    KeysymChar {
        keysym: 0x0037,
        char: '7',
    },
    KeysymChar {
        keysym: 0x0038,
        char: '8',
    },
    KeysymChar {
        keysym: 0x0039,
        char: '9',
    },
    KeysymChar {
        keysym: 0x003a,
        char: ':',
    },
    KeysymChar {
        keysym: 0x003b,
        char: ';',
    },
    KeysymChar {
        keysym: 0x003c,
        char: '<',
    },
    KeysymChar {
        keysym: 0x003d,
        char: '=',
    },
    KeysymChar {
        keysym: 0x003e,
        char: '>',
    },
    KeysymChar {
        keysym: 0x003f,
        char: '?',
    },
    KeysymChar {
        keysym: 0x0040,
        char: '@',
    },
    KeysymChar {
        keysym: 0x0041,
        char: 'A',
    },
    KeysymChar {
        keysym: 0x0042,
        char: 'B',
    },
    KeysymChar {
        keysym: 0x0043,
        char: 'C',
    },
    KeysymChar {
        keysym: 0x0044,
        char: 'D',
    },
    KeysymChar {
        keysym: 0x0045,
        char: 'E',
    },
    KeysymChar {
        keysym: 0x0046,
        char: 'F',
    },
    KeysymChar {
        keysym: 0x0047,
        char: 'G',
    },
    KeysymChar {
        keysym: 0x0048,
        char: 'H',
    },
    KeysymChar {
        keysym: 0x0049,
        char: 'I',
    },
    KeysymChar {
        keysym: 0x004a,
        char: 'J',
    },
    KeysymChar {
        keysym: 0x004b,
        char: 'K',
    },
    KeysymChar {
        keysym: 0x004c,
        char: 'L',
    },
    KeysymChar {
        keysym: 0x004d,
        char: 'M',
    },
    KeysymChar {
        keysym: 0x004e,
        char: 'N',
    },
    KeysymChar {
        keysym: 0x004f,
        char: 'O',
    },
    KeysymChar {
        keysym: 0x0050,
        char: 'P',
    },
    KeysymChar {
        keysym: 0x0051,
        char: 'Q',
    },
    KeysymChar {
        keysym: 0x0052,
        char: 'R',
    },
    KeysymChar {
        keysym: 0x0053,
        char: 'S',
    },
    KeysymChar {
        keysym: 0x0054,
        char: 'T',
    },
    KeysymChar {
        keysym: 0x0055,
        char: 'U',
    },
    KeysymChar {
        keysym: 0x0056,
        char: 'V',
    },
    KeysymChar {
        keysym: 0x0057,
        char: 'W',
    },
    KeysymChar {
        keysym: 0x0058,
        char: 'X',
    },
    KeysymChar {
        keysym: 0x0059,
        char: 'Y',
    },
    KeysymChar {
        keysym: 0x005a,
        char: 'Z',
    },
    KeysymChar {
        keysym: 0x005b,
        char: '[',
    },
    KeysymChar {
        keysym: 0x005c,
        char: '\\',
    },
    KeysymChar {
        keysym: 0x005d,
        char: ']',
    },
    KeysymChar {
        keysym: 0x005e,
        char: '^',
    },
    KeysymChar {
        keysym: 0x005f,
        char: '_',
    },
    KeysymChar {
        keysym: 0x0060,
        char: '`',
    },
    KeysymChar {
        keysym: 0x0061,
        char: 'a',
    },
    KeysymChar {
        keysym: 0x0062,
        char: 'b',
    },
    KeysymChar {
        keysym: 0x0063,
        char: 'c',
    },
    KeysymChar {
        keysym: 0x0064,
        char: 'd',
    },
    KeysymChar {
        keysym: 0x0065,
        char: 'e',
    },
    KeysymChar {
        keysym: 0x0066,
        char: 'f',
    },
    KeysymChar {
        keysym: 0x0067,
        char: 'g',
    },
    KeysymChar {
        keysym: 0x0068,
        char: 'h',
    },
    KeysymChar {
        keysym: 0x0069,
        char: 'i',
    },
    KeysymChar {
        keysym: 0x006a,
        char: 'j',
    },
    KeysymChar {
        keysym: 0x006b,
        char: 'k',
    },
    KeysymChar {
        keysym: 0x006c,
        char: 'l',
    },
    KeysymChar {
        keysym: 0x006d,
        char: 'm',
    },
    KeysymChar {
        keysym: 0x006e,
        char: 'n',
    },
    KeysymChar {
        keysym: 0x006f,
        char: 'o',
    },
    KeysymChar {
        keysym: 0x0070,
        char: 'p',
    },
    KeysymChar {
        keysym: 0x0071,
        char: 'q',
    },
    KeysymChar {
        keysym: 0x0072,
        char: 'r',
    },
    KeysymChar {
        keysym: 0x0073,
        char: 's',
    },
    KeysymChar {
        keysym: 0x0074,
        char: 't',
    },
    KeysymChar {
        keysym: 0x0075,
        char: 'u',
    },
    KeysymChar {
        keysym: 0x0076,
        char: 'v',
    },
    KeysymChar {
        keysym: 0x0077,
        char: 'w',
    },
    KeysymChar {
        keysym: 0x0078,
        char: 'x',
    },
    KeysymChar {
        keysym: 0x0079,
        char: 'y',
    },
    KeysymChar {
        keysym: 0x007a,
        char: 'z',
    },
    KeysymChar {
        keysym: 0x007b,
        char: '{',
    },
    KeysymChar {
        keysym: 0x007c,
        char: '|',
    },
    KeysymChar {
        keysym: 0x007d,
        char: '}',
    },
    KeysymChar {
        keysym: 0x007e,
        char: '~',
    },
    KeysymChar {
        keysym: 0x00a0,
        char: '\u{a0}',
    },
    KeysymChar {
        keysym: 0x00a1,
        char: '¡',
    },
    KeysymChar {
        keysym: 0x00a2,
        char: '¢',
    },
    KeysymChar {
        keysym: 0x00a3,
        char: '£',
    },
    KeysymChar {
        keysym: 0x00a4,
        char: '¤',
    },
    KeysymChar {
        keysym: 0x00a5,
        char: '¥',
    },
    KeysymChar {
        keysym: 0x00a6,
        char: '¦',
    },
    KeysymChar {
        keysym: 0x00a7,
        char: '§',
    },
    KeysymChar {
        keysym: 0x00a8,
        char: '¨',
    },
    KeysymChar {
        keysym: 0x00a9,
        char: '©',
    },
    KeysymChar {
        keysym: 0x00aa,
        char: 'ª',
    },
    KeysymChar {
        keysym: 0x00ab,
        char: '«',
    },
    KeysymChar {
        keysym: 0x00ac,
        char: '¬',
    },
    KeysymChar {
        keysym: 0x00ad,
        char: '\u{ad}',
    },
    KeysymChar {
        keysym: 0x00ae,
        char: '®',
    },
    KeysymChar {
        keysym: 0x00af,
        char: '¯',
    },
    KeysymChar {
        keysym: 0x00b0,
        char: '°',
    },
    KeysymChar {
        keysym: 0x00b1,
        char: '±',
    },
    KeysymChar {
        keysym: 0x00b2,
        char: '²',
    },
    KeysymChar {
        keysym: 0x00b3,
        char: '³',
    },
    KeysymChar {
        keysym: 0x00b4,
        char: '´',
    },
    KeysymChar {
        keysym: 0x00b5,
        char: 'µ',
    },
    KeysymChar {
        keysym: 0x00b6,
        char: '¶',
    },
    KeysymChar {
        keysym: 0x00b7,
        char: '·',
    },
    KeysymChar {
        keysym: 0x00b8,
        char: '¸',
    },
    KeysymChar {
        keysym: 0x00b9,
        char: '¹',
    },
    KeysymChar {
        keysym: 0x00ba,
        char: 'º',
    },
    KeysymChar {
        keysym: 0x00bb,
        char: '»',
    },
    KeysymChar {
        keysym: 0x00bc,
        char: '¼',
    },
    KeysymChar {
        keysym: 0x00bd,
        char: '½',
    },
    KeysymChar {
        keysym: 0x00be,
        char: '¾',
    },
    KeysymChar {
        keysym: 0x00bf,
        char: '¿',
    },
    KeysymChar {
        keysym: 0x00c0,
        char: 'À',
    },
    KeysymChar {
        keysym: 0x00c1,
        char: 'Á',
    },
    KeysymChar {
        keysym: 0x00c2,
        char: 'Â',
    },
    KeysymChar {
        keysym: 0x00c3,
        char: 'Ã',
    },
    KeysymChar {
        keysym: 0x00c4,
        char: 'Ä',
    },
    KeysymChar {
        keysym: 0x00c5,
        char: 'Å',
    },
    KeysymChar {
        keysym: 0x00c6,
        char: 'Æ',
    },
    KeysymChar {
        keysym: 0x00c7,
        char: 'Ç',
    },
    KeysymChar {
        keysym: 0x00c8,
        char: 'È',
    },
    KeysymChar {
        keysym: 0x00c9,
        char: 'É',
    },
    KeysymChar {
        keysym: 0x00ca,
        char: 'Ê',
    },
    KeysymChar {
        keysym: 0x00cb,
        char: 'Ë',
    },
    KeysymChar {
        keysym: 0x00cc,
        char: 'Ì',
    },
    KeysymChar {
        keysym: 0x00cd,
        char: 'Í',
    },
    KeysymChar {
        keysym: 0x00ce,
        char: 'Î',
    },
    KeysymChar {
        keysym: 0x00cf,
        char: 'Ï',
    },
    KeysymChar {
        keysym: 0x00d0,
        char: 'Ð',
    },
    KeysymChar {
        keysym: 0x00d1,
        char: 'Ñ',
    },
    KeysymChar {
        keysym: 0x00d2,
        char: 'Ò',
    },
    KeysymChar {
        keysym: 0x00d3,
        char: 'Ó',
    },
    KeysymChar {
        keysym: 0x00d4,
        char: 'Ô',
    },
    KeysymChar {
        keysym: 0x00d5,
        char: 'Õ',
    },
    KeysymChar {
        keysym: 0x00d6,
        char: 'Ö',
    },
    KeysymChar {
        keysym: 0x00d7,
        char: '×',
    },
    KeysymChar {
        keysym: 0x00d8,
        char: 'Ø',
    },
    KeysymChar {
        keysym: 0x00d9,
        char: 'Ù',
    },
    KeysymChar {
        keysym: 0x00da,
        char: 'Ú',
    },
    KeysymChar {
        keysym: 0x00db,
        char: 'Û',
    },
    KeysymChar {
        keysym: 0x00dc,
        char: 'Ü',
    },
    KeysymChar {
        keysym: 0x00dd,
        char: 'Ý',
    },
    KeysymChar {
        keysym: 0x00de,
        char: 'Þ',
    },
    KeysymChar {
        keysym: 0x00df,
        char: 'ß',
    },
    KeysymChar {
        keysym: 0x00e0,
        char: 'à',
    },
    KeysymChar {
        keysym: 0x00e1,
        char: 'á',
    },
    KeysymChar {
        keysym: 0x00e2,
        char: 'â',
    },
    KeysymChar {
        keysym: 0x00e3,
        char: 'ã',
    },
    KeysymChar {
        keysym: 0x00e4,
        char: 'ä',
    },
    KeysymChar {
        keysym: 0x00e5,
        char: 'å',
    },
    KeysymChar {
        keysym: 0x00e6,
        char: 'æ',
    },
    KeysymChar {
        keysym: 0x00e7,
        char: 'ç',
    },
    KeysymChar {
        keysym: 0x00e8,
        char: 'è',
    },
    KeysymChar {
        keysym: 0x00e9,
        char: 'é',
    },
    KeysymChar {
        keysym: 0x00ea,
        char: 'ê',
    },
    KeysymChar {
        keysym: 0x00eb,
        char: 'ë',
    },
    KeysymChar {
        keysym: 0x00ec,
        char: 'ì',
    },
    KeysymChar {
        keysym: 0x00ed,
        char: 'í',
    },
    KeysymChar {
        keysym: 0x00ee,
        char: 'î',
    },
    KeysymChar {
        keysym: 0x00ef,
        char: 'ï',
    },
    KeysymChar {
        keysym: 0x00f0,
        char: 'ð',
    },
    KeysymChar {
        keysym: 0x00f1,
        char: 'ñ',
    },
    KeysymChar {
        keysym: 0x00f2,
        char: 'ò',
    },
    KeysymChar {
        keysym: 0x00f3,
        char: 'ó',
    },
    KeysymChar {
        keysym: 0x00f4,
        char: 'ô',
    },
    KeysymChar {
        keysym: 0x00f5,
        char: 'õ',
    },
    KeysymChar {
        keysym: 0x00f6,
        char: 'ö',
    },
    KeysymChar {
        keysym: 0x00f7,
        char: '÷',
    },
    KeysymChar {
        keysym: 0x00f8,
        char: 'ø',
    },
    KeysymChar {
        keysym: 0x00f9,
        char: 'ù',
    },
    KeysymChar {
        keysym: 0x00fa,
        char: 'ú',
    },
    KeysymChar {
        keysym: 0x00fb,
        char: 'û',
    },
    KeysymChar {
        keysym: 0x00fc,
        char: 'ü',
    },
    KeysymChar {
        keysym: 0x00fd,
        char: 'ý',
    },
    KeysymChar {
        keysym: 0x00fe,
        char: 'þ',
    },
    KeysymChar {
        keysym: 0x00ff,
        char: 'ÿ',
    },
    KeysymChar {
        keysym: 0x01a1,
        char: 'Ą',
    },
    KeysymChar {
        keysym: 0x01a2,
        char: '˘',
    },
    KeysymChar {
        keysym: 0x01a3,
        char: 'Ł',
    },
    KeysymChar {
        keysym: 0x01a5,
        char: 'Ľ',
    },
    KeysymChar {
        keysym: 0x01a6,
        char: 'Ś',
    },
    KeysymChar {
        keysym: 0x01a9,
        char: 'Š',
    },
    KeysymChar {
        keysym: 0x01aa,
        char: 'Ş',
    },
    KeysymChar {
        keysym: 0x01ab,
        char: 'Ť',
    },
    KeysymChar {
        keysym: 0x01ac,
        char: 'Ź',
    },
    KeysymChar {
        keysym: 0x01ae,
        char: 'Ž',
    },
    KeysymChar {
        keysym: 0x01af,
        char: 'Ż',
    },
    KeysymChar {
        keysym: 0x01b1,
        char: 'ą',
    },
    KeysymChar {
        keysym: 0x01b2,
        char: '˛',
    },
    KeysymChar {
        keysym: 0x01b3,
        char: 'ł',
    },
    KeysymChar {
        keysym: 0x01b5,
        char: 'ľ',
    },
    KeysymChar {
        keysym: 0x01b6,
        char: 'ś',
    },
    KeysymChar {
        keysym: 0x01b7,
        char: 'ˇ',
    },
    KeysymChar {
        keysym: 0x01b9,
        char: 'š',
    },
    KeysymChar {
        keysym: 0x01ba,
        char: 'ş',
    },
    KeysymChar {
        keysym: 0x01bb,
        char: 'ť',
    },
    KeysymChar {
        keysym: 0x01bc,
        char: 'ź',
    },
    KeysymChar {
        keysym: 0x01bd,
        char: '˝',
    },
    KeysymChar {
        keysym: 0x01be,
        char: 'ž',
    },
    KeysymChar {
        keysym: 0x01bf,
        char: 'ż',
    },
    KeysymChar {
        keysym: 0x01c0,
        char: 'Ŕ',
    },
    KeysymChar {
        keysym: 0x01c3,
        char: 'Ă',
    },
    KeysymChar {
        keysym: 0x01c5,
        char: 'Ĺ',
    },
    KeysymChar {
        keysym: 0x01c6,
        char: 'Ć',
    },
    KeysymChar {
        keysym: 0x01c8,
        char: 'Č',
    },
    KeysymChar {
        keysym: 0x01ca,
        char: 'Ę',
    },
    KeysymChar {
        keysym: 0x01cc,
        char: 'Ě',
    },
    KeysymChar {
        keysym: 0x01cf,
        char: 'Ď',
    },
    KeysymChar {
        keysym: 0x01d0,
        char: 'Đ',
    },
    KeysymChar {
        keysym: 0x01d1,
        char: 'Ń',
    },
    KeysymChar {
        keysym: 0x01d2,
        char: 'Ň',
    },
    KeysymChar {
        keysym: 0x01d5,
        char: 'Ő',
    },
    KeysymChar {
        keysym: 0x01d8,
        char: 'Ř',
    },
    KeysymChar {
        keysym: 0x01d9,
        char: 'Ů',
    },
    KeysymChar {
        keysym: 0x01db,
        char: 'Ű',
    },
    KeysymChar {
        keysym: 0x01de,
        char: 'Ţ',
    },
    KeysymChar {
        keysym: 0x01e0,
        char: 'ŕ',
    },
    KeysymChar {
        keysym: 0x01e3,
        char: 'ă',
    },
    KeysymChar {
        keysym: 0x01e5,
        char: 'ĺ',
    },
    KeysymChar {
        keysym: 0x01e6,
        char: 'ć',
    },
    KeysymChar {
        keysym: 0x01e8,
        char: 'č',
    },
    KeysymChar {
        keysym: 0x01ea,
        char: 'ę',
    },
    KeysymChar {
        keysym: 0x01ec,
        char: 'ě',
    },
    KeysymChar {
        keysym: 0x01ef,
        char: 'ď',
    },
    KeysymChar {
        keysym: 0x01f0,
        char: 'đ',
    },
    KeysymChar {
        keysym: 0x01f1,
        char: 'ń',
    },
    KeysymChar {
        keysym: 0x01f2,
        char: 'ň',
    },
    KeysymChar {
        keysym: 0x01f5,
        char: 'ő',
    },
    KeysymChar {
        keysym: 0x01f8,
        char: 'ř',
    },
    KeysymChar {
        keysym: 0x01f9,
        char: 'ů',
    },
    KeysymChar {
        keysym: 0x01fb,
        char: 'ű',
    },
    KeysymChar {
        keysym: 0x01fe,
        char: 'ţ',
    },
    KeysymChar {
        keysym: 0x01ff,
        char: '˙',
    },
    KeysymChar {
        keysym: 0x02a1,
        char: 'Ħ',
    },
    KeysymChar {
        keysym: 0x02a6,
        char: 'Ĥ',
    },
    KeysymChar {
        keysym: 0x02a9,
        char: 'İ',
    },
    KeysymChar {
        keysym: 0x02ab,
        char: 'Ğ',
    },
    KeysymChar {
        keysym: 0x02ac,
        char: 'Ĵ',
    },
    KeysymChar {
        keysym: 0x02b1,
        char: 'ħ',
    },
    KeysymChar {
        keysym: 0x02b6,
        char: 'ĥ',
    },
    KeysymChar {
        keysym: 0x02b9,
        char: 'ı',
    },
    KeysymChar {
        keysym: 0x02bb,
        char: 'ğ',
    },
    KeysymChar {
        keysym: 0x02bc,
        char: 'ĵ',
    },
    KeysymChar {
        keysym: 0x02c5,
        char: 'Ċ',
    },
    KeysymChar {
        keysym: 0x02c6,
        char: 'Ĉ',
    },
    KeysymChar {
        keysym: 0x02d5,
        char: 'Ġ',
    },
    KeysymChar {
        keysym: 0x02d8,
        char: 'Ĝ',
    },
    KeysymChar {
        keysym: 0x02dd,
        char: 'Ŭ',
    },
    KeysymChar {
        keysym: 0x02de,
        char: 'Ŝ',
    },
    KeysymChar {
        keysym: 0x02e5,
        char: 'ċ',
    },
    KeysymChar {
        keysym: 0x02e6,
        char: 'ĉ',
    },
    KeysymChar {
        keysym: 0x02f5,
        char: 'ġ',
    },
    KeysymChar {
        keysym: 0x02f8,
        char: 'ĝ',
    },
    KeysymChar {
        keysym: 0x02fd,
        char: 'ŭ',
    },
    KeysymChar {
        keysym: 0x02fe,
        char: 'ŝ',
    },
    KeysymChar {
        keysym: 0x03a2,
        char: 'ĸ',
    },
    KeysymChar {
        keysym: 0x03a3,
        char: 'Ŗ',
    },
    KeysymChar {
        keysym: 0x03a5,
        char: 'Ĩ',
    },
    KeysymChar {
        keysym: 0x03a6,
        char: 'Ļ',
    },
    KeysymChar {
        keysym: 0x03aa,
        char: 'Ē',
    },
    KeysymChar {
        keysym: 0x03ab,
        char: 'Ģ',
    },
    KeysymChar {
        keysym: 0x03ac,
        char: 'Ŧ',
    },
    KeysymChar {
        keysym: 0x03b3,
        char: 'ŗ',
    },
    KeysymChar {
        keysym: 0x03b5,
        char: 'ĩ',
    },
    KeysymChar {
        keysym: 0x03b6,
        char: 'ļ',
    },
    KeysymChar {
        keysym: 0x03ba,
        char: 'ē',
    },
    KeysymChar {
        keysym: 0x03bb,
        char: 'ģ',
    },
    KeysymChar {
        keysym: 0x03bc,
        char: 'ŧ',
    },
    KeysymChar {
        keysym: 0x03bd,
        char: 'Ŋ',
    },
    KeysymChar {
        keysym: 0x03bf,
        char: 'ŋ',
    },
    KeysymChar {
        keysym: 0x03c0,
        char: 'Ā',
    },
    KeysymChar {
        keysym: 0x03c7,
        char: 'Į',
    },
    KeysymChar {
        keysym: 0x03cc,
        char: 'Ė',
    },
    KeysymChar {
        keysym: 0x03cf,
        char: 'Ī',
    },
    KeysymChar {
        keysym: 0x03d1,
        char: 'Ņ',
    },
    KeysymChar {
        keysym: 0x03d2,
        char: 'Ō',
    },
    KeysymChar {
        keysym: 0x03d3,
        char: 'Ķ',
    },
    KeysymChar {
        keysym: 0x03d9,
        char: 'Ų',
    },
    KeysymChar {
        keysym: 0x03dd,
        char: 'Ũ',
    },
    KeysymChar {
        keysym: 0x03de,
        char: 'Ū',
    },
    KeysymChar {
        keysym: 0x03e0,
        char: 'ā',
    },
    KeysymChar {
        keysym: 0x03e7,
        char: 'į',
    },
    KeysymChar {
        keysym: 0x03ec,
        char: 'ė',
    },
    KeysymChar {
        keysym: 0x03ef,
        char: 'ī',
    },
    KeysymChar {
        keysym: 0x03f1,
        char: 'ņ',
    },
    KeysymChar {
        keysym: 0x03f2,
        char: 'ō',
    },
    KeysymChar {
        keysym: 0x03f3,
        char: 'ķ',
    },
    KeysymChar {
        keysym: 0x03f9,
        char: 'ų',
    },
    KeysymChar {
        keysym: 0x03fd,
        char: 'ũ',
    },
    KeysymChar {
        keysym: 0x03fe,
        char: 'ū',
    },
    KeysymChar {
        keysym: 0x047e,
        char: '‾',
    },
    KeysymChar {
        keysym: 0x04a1,
        char: '。',
    },
    KeysymChar {
        keysym: 0x04a2,
        char: '「',
    },
    KeysymChar {
        keysym: 0x04a3,
        char: '」',
    },
    KeysymChar {
        keysym: 0x04a4,
        char: '、',
    },
    KeysymChar {
        keysym: 0x04a5,
        char: '・',
    },
    KeysymChar {
        keysym: 0x04a6,
        char: 'ヲ',
    },
    KeysymChar {
        keysym: 0x04a7,
        char: 'ァ',
    },
    KeysymChar {
        keysym: 0x04a8,
        char: 'ィ',
    },
    KeysymChar {
        keysym: 0x04a9,
        char: 'ゥ',
    },
    KeysymChar {
        keysym: 0x04aa,
        char: 'ェ',
    },
    KeysymChar {
        keysym: 0x04ab,
        char: 'ォ',
    },
    KeysymChar {
        keysym: 0x04ac,
        char: 'ャ',
    },
    KeysymChar {
        keysym: 0x04ad,
        char: 'ュ',
    },
    KeysymChar {
        keysym: 0x04ae,
        char: 'ョ',
    },
    KeysymChar {
        keysym: 0x04af,
        char: 'ッ',
    },
    KeysymChar {
        keysym: 0x04b0,
        char: 'ー',
    },
    KeysymChar {
        keysym: 0x04b1,
        char: 'ア',
    },
    KeysymChar {
        keysym: 0x04b2,
        char: 'イ',
    },
    KeysymChar {
        keysym: 0x04b3,
        char: 'ウ',
    },
    KeysymChar {
        keysym: 0x04b4,
        char: 'エ',
    },
    KeysymChar {
        keysym: 0x04b5,
        char: 'オ',
    },
    KeysymChar {
        keysym: 0x04b6,
        char: 'カ',
    },
    KeysymChar {
        keysym: 0x04b7,
        char: 'キ',
    },
    KeysymChar {
        keysym: 0x04b8,
        char: 'ク',
    },
    KeysymChar {
        keysym: 0x04b9,
        char: 'ケ',
    },
    KeysymChar {
        keysym: 0x04ba,
        char: 'コ',
    },
    KeysymChar {
        keysym: 0x04bb,
        char: 'サ',
    },
    KeysymChar {
        keysym: 0x04bc,
        char: 'シ',
    },
    KeysymChar {
        keysym: 0x04bd,
        char: 'ス',
    },
    KeysymChar {
        keysym: 0x04be,
        char: 'セ',
    },
    KeysymChar {
        keysym: 0x04bf,
        char: 'ソ',
    },
    KeysymChar {
        keysym: 0x04c0,
        char: 'タ',
    },
    KeysymChar {
        keysym: 0x04c1,
        char: 'チ',
    },
    KeysymChar {
        keysym: 0x04c2,
        char: 'ツ',
    },
    KeysymChar {
        keysym: 0x04c3,
        char: 'テ',
    },
    KeysymChar {
        keysym: 0x04c4,
        char: 'ト',
    },
    KeysymChar {
        keysym: 0x04c5,
        char: 'ナ',
    },
    KeysymChar {
        keysym: 0x04c6,
        char: 'ニ',
    },
    KeysymChar {
        keysym: 0x04c7,
        char: 'ヌ',
    },
    KeysymChar {
        keysym: 0x04c8,
        char: 'ネ',
    },
    KeysymChar {
        keysym: 0x04c9,
        char: 'ノ',
    },
    KeysymChar {
        keysym: 0x04ca,
        char: 'ハ',
    },
    KeysymChar {
        keysym: 0x04cb,
        char: 'ヒ',
    },
    KeysymChar {
        keysym: 0x04cc,
        char: 'フ',
    },
    KeysymChar {
        keysym: 0x04cd,
        char: 'ヘ',
    },
    KeysymChar {
        keysym: 0x04ce,
        char: 'ホ',
    },
    KeysymChar {
        keysym: 0x04cf,
        char: 'マ',
    },
    KeysymChar {
        keysym: 0x04d0,
        char: 'ミ',
    },
    KeysymChar {
        keysym: 0x04d1,
        char: 'ム',
    },
    KeysymChar {
        keysym: 0x04d2,
        char: 'メ',
    },
    KeysymChar {
        keysym: 0x04d3,
        char: 'モ',
    },
    KeysymChar {
        keysym: 0x04d4,
        char: 'ヤ',
    },
    KeysymChar {
        keysym: 0x04d5,
        char: 'ユ',
    },
    KeysymChar {
        keysym: 0x04d6,
        char: 'ヨ',
    },
    KeysymChar {
        keysym: 0x04d7,
        char: 'ラ',
    },
    KeysymChar {
        keysym: 0x04d8,
        char: 'リ',
    },
    KeysymChar {
        keysym: 0x04d9,
        char: 'ル',
    },
    KeysymChar {
        keysym: 0x04da,
        char: 'レ',
    },
    KeysymChar {
        keysym: 0x04db,
        char: 'ロ',
    },
    KeysymChar {
        keysym: 0x04dc,
        char: 'ワ',
    },
    KeysymChar {
        keysym: 0x04dd,
        char: 'ン',
    },
    KeysymChar {
        keysym: 0x04de,
        char: '゛',
    },
    KeysymChar {
        keysym: 0x04df,
        char: '゜',
    },
    KeysymChar {
        keysym: 0x05ac,
        char: '،',
    },
    KeysymChar {
        keysym: 0x05bb,
        char: '؛',
    },
    KeysymChar {
        keysym: 0x05bf,
        char: '؟',
    },
    KeysymChar {
        keysym: 0x05c1,
        char: 'ء',
    },
    KeysymChar {
        keysym: 0x05c2,
        char: 'آ',
    },
    KeysymChar {
        keysym: 0x05c3,
        char: 'أ',
    },
    KeysymChar {
        keysym: 0x05c4,
        char: 'ؤ',
    },
    KeysymChar {
        keysym: 0x05c5,
        char: 'إ',
    },
    KeysymChar {
        keysym: 0x05c6,
        char: 'ئ',
    },
    KeysymChar {
        keysym: 0x05c7,
        char: 'ا',
    },
    KeysymChar {
        keysym: 0x05c8,
        char: 'ب',
    },
    KeysymChar {
        keysym: 0x05c9,
        char: 'ة',
    },
    KeysymChar {
        keysym: 0x05ca,
        char: 'ت',
    },
    KeysymChar {
        keysym: 0x05cb,
        char: 'ث',
    },
    KeysymChar {
        keysym: 0x05cc,
        char: 'ج',
    },
    KeysymChar {
        keysym: 0x05cd,
        char: 'ح',
    },
    KeysymChar {
        keysym: 0x05ce,
        char: 'خ',
    },
    KeysymChar {
        keysym: 0x05cf,
        char: 'د',
    },
    KeysymChar {
        keysym: 0x05d0,
        char: 'ذ',
    },
    KeysymChar {
        keysym: 0x05d1,
        char: 'ر',
    },
    KeysymChar {
        keysym: 0x05d2,
        char: 'ز',
    },
    KeysymChar {
        keysym: 0x05d3,
        char: 'س',
    },
    KeysymChar {
        keysym: 0x05d4,
        char: 'ش',
    },
    KeysymChar {
        keysym: 0x05d5,
        char: 'ص',
    },
    KeysymChar {
        keysym: 0x05d6,
        char: 'ض',
    },
    KeysymChar {
        keysym: 0x05d7,
        char: 'ط',
    },
    KeysymChar {
        keysym: 0x05d8,
        char: 'ظ',
    },
    KeysymChar {
        keysym: 0x05d9,
        char: 'ع',
    },
    KeysymChar {
        keysym: 0x05da,
        char: 'غ',
    },
    KeysymChar {
        keysym: 0x05e0,
        char: 'ـ',
    },
    KeysymChar {
        keysym: 0x05e1,
        char: 'ف',
    },
    KeysymChar {
        keysym: 0x05e2,
        char: 'ق',
    },
    KeysymChar {
        keysym: 0x05e3,
        char: 'ك',
    },
    KeysymChar {
        keysym: 0x05e4,
        char: 'ل',
    },
    KeysymChar {
        keysym: 0x05e5,
        char: 'م',
    },
    KeysymChar {
        keysym: 0x05e6,
        char: 'ن',
    },
    KeysymChar {
        keysym: 0x05e7,
        char: 'ه',
    },
    KeysymChar {
        keysym: 0x05e8,
        char: 'و',
    },
    KeysymChar {
        keysym: 0x05e9,
        char: 'ى',
    },
    KeysymChar {
        keysym: 0x05ea,
        char: 'ي',
    },
    KeysymChar {
        keysym: 0x05eb,
        char: '\u{64b}',
    },
    KeysymChar {
        keysym: 0x05ec,
        char: '\u{64c}',
    },
    KeysymChar {
        keysym: 0x05ed,
        char: '\u{64d}',
    },
    KeysymChar {
        keysym: 0x05ee,
        char: '\u{64e}',
    },
    KeysymChar {
        keysym: 0x05ef,
        char: '\u{64f}',
    },
    KeysymChar {
        keysym: 0x05f0,
        char: '\u{650}',
    },
    KeysymChar {
        keysym: 0x05f1,
        char: '\u{651}',
    },
    KeysymChar {
        keysym: 0x05f2,
        char: '\u{652}',
    },
    KeysymChar {
        keysym: 0x06a1,
        char: 'ђ',
    },
    KeysymChar {
        keysym: 0x06a2,
        char: 'ѓ',
    },
    KeysymChar {
        keysym: 0x06a3,
        char: 'ё',
    },
    KeysymChar {
        keysym: 0x06a4,
        char: 'є',
    },
    KeysymChar {
        keysym: 0x06a5,
        char: 'ѕ',
    },
    KeysymChar {
        keysym: 0x06a6,
        char: 'і',
    },
    KeysymChar {
        keysym: 0x06a7,
        char: 'ї',
    },
    KeysymChar {
        keysym: 0x06a8,
        char: 'ј',
    },
    KeysymChar {
        keysym: 0x06a9,
        char: 'љ',
    },
    KeysymChar {
        keysym: 0x06aa,
        char: 'њ',
    },
    KeysymChar {
        keysym: 0x06ab,
        char: 'ћ',
    },
    KeysymChar {
        keysym: 0x06ac,
        char: 'ќ',
    },
    KeysymChar {
        keysym: 0x06ad,
        char: 'ґ',
    },
    KeysymChar {
        keysym: 0x06ae,
        char: 'ў',
    },
    KeysymChar {
        keysym: 0x06af,
        char: 'џ',
    },
    KeysymChar {
        keysym: 0x06b0,
        char: '№',
    },
    KeysymChar {
        keysym: 0x06b1,
        char: 'Ђ',
    },
    KeysymChar {
        keysym: 0x06b2,
        char: 'Ѓ',
    },
    KeysymChar {
        keysym: 0x06b3,
        char: 'Ё',
    },
    KeysymChar {
        keysym: 0x06b4,
        char: 'Є',
    },
    KeysymChar {
        keysym: 0x06b5,
        char: 'Ѕ',
    },
    KeysymChar {
        keysym: 0x06b6,
        char: 'І',
    },
    KeysymChar {
        keysym: 0x06b7,
        char: 'Ї',
    },
    KeysymChar {
        keysym: 0x06b8,
        char: 'Ј',
    },
    KeysymChar {
        keysym: 0x06b9,
        char: 'Љ',
    },
    KeysymChar {
        keysym: 0x06ba,
        char: 'Њ',
    },
    KeysymChar {
        keysym: 0x06bb,
        char: 'Ћ',
    },
    KeysymChar {
        keysym: 0x06bc,
        char: 'Ќ',
    },
    KeysymChar {
        keysym: 0x06bd,
        char: 'Ґ',
    },
    KeysymChar {
        keysym: 0x06be,
        char: 'Ў',
    },
    KeysymChar {
        keysym: 0x06bf,
        char: 'Џ',
    },
    KeysymChar {
        keysym: 0x06c0,
        char: 'ю',
    },
    KeysymChar {
        keysym: 0x06c1,
        char: 'а',
    },
    KeysymChar {
        keysym: 0x06c2,
        char: 'б',
    },
    KeysymChar {
        keysym: 0x06c3,
        char: 'ц',
    },
    KeysymChar {
        keysym: 0x06c4,
        char: 'д',
    },
    KeysymChar {
        keysym: 0x06c5,
        char: 'е',
    },
    KeysymChar {
        keysym: 0x06c6,
        char: 'ф',
    },
    KeysymChar {
        keysym: 0x06c7,
        char: 'г',
    },
    KeysymChar {
        keysym: 0x06c8,
        char: 'х',
    },
    KeysymChar {
        keysym: 0x06c9,
        char: 'и',
    },
    KeysymChar {
        keysym: 0x06ca,
        char: 'й',
    },
    KeysymChar {
        keysym: 0x06cb,
        char: 'к',
    },
    KeysymChar {
        keysym: 0x06cc,
        char: 'л',
    },
    KeysymChar {
        keysym: 0x06cd,
        char: 'м',
    },
    KeysymChar {
        keysym: 0x06ce,
        char: 'н',
    },
    KeysymChar {
        keysym: 0x06cf,
        char: 'о',
    },
    KeysymChar {
        keysym: 0x06d0,
        char: 'п',
    },
    KeysymChar {
        keysym: 0x06d1,
        char: 'я',
    },
    KeysymChar {
        keysym: 0x06d2,
        char: 'р',
    },
    KeysymChar {
        keysym: 0x06d3,
        char: 'с',
    },
    KeysymChar {
        keysym: 0x06d4,
        char: 'т',
    },
    KeysymChar {
        keysym: 0x06d5,
        char: 'у',
    },
    KeysymChar {
        keysym: 0x06d6,
        char: 'ж',
    },
    KeysymChar {
        keysym: 0x06d7,
        char: 'в',
    },
    KeysymChar {
        keysym: 0x06d8,
        char: 'ь',
    },
    KeysymChar {
        keysym: 0x06d9,
        char: 'ы',
    },
    KeysymChar {
        keysym: 0x06da,
        char: 'з',
    },
    KeysymChar {
        keysym: 0x06db,
        char: 'ш',
    },
    KeysymChar {
        keysym: 0x06dc,
        char: 'э',
    },
    KeysymChar {
        keysym: 0x06dd,
        char: 'щ',
    },
    KeysymChar {
        keysym: 0x06de,
        char: 'ч',
    },
    KeysymChar {
        keysym: 0x06df,
        char: 'ъ',
    },
    KeysymChar {
        keysym: 0x06e0,
        char: 'Ю',
    },
    KeysymChar {
        keysym: 0x06e1,
        char: 'А',
    },
    KeysymChar {
        keysym: 0x06e2,
        char: 'Б',
    },
    KeysymChar {
        keysym: 0x06e3,
        char: 'Ц',
    },
    KeysymChar {
        keysym: 0x06e4,
        char: 'Д',
    },
    KeysymChar {
        keysym: 0x06e5,
        char: 'Е',
    },
    KeysymChar {
        keysym: 0x06e6,
        char: 'Ф',
    },
    KeysymChar {
        keysym: 0x06e7,
        char: 'Г',
    },
    KeysymChar {
        keysym: 0x06e8,
        char: 'Х',
    },
    KeysymChar {
        keysym: 0x06e9,
        char: 'И',
    },
    KeysymChar {
        keysym: 0x06ea,
        char: 'Й',
    },
    KeysymChar {
        keysym: 0x06eb,
        char: 'К',
    },
    KeysymChar {
        keysym: 0x06ec,
        char: 'Л',
    },
    KeysymChar {
        keysym: 0x06ed,
        char: 'М',
    },
    KeysymChar {
        keysym: 0x06ee,
        char: 'Н',
    },
    KeysymChar {
        keysym: 0x06ef,
        char: 'О',
    },
    KeysymChar {
        keysym: 0x06f0,
        char: 'П',
    },
    KeysymChar {
        keysym: 0x06f1,
        char: 'Я',
    },
    KeysymChar {
        keysym: 0x06f2,
        char: 'Р',
    },
    KeysymChar {
        keysym: 0x06f3,
        char: 'С',
    },
    KeysymChar {
        keysym: 0x06f4,
        char: 'Т',
    },
    KeysymChar {
        keysym: 0x06f5,
        char: 'У',
    },
    KeysymChar {
        keysym: 0x06f6,
        char: 'Ж',
    },
    KeysymChar {
        keysym: 0x06f7,
        char: 'В',
    },
    KeysymChar {
        keysym: 0x06f8,
        char: 'Ь',
    },
    KeysymChar {
        keysym: 0x06f9,
        char: 'Ы',
    },
    KeysymChar {
        keysym: 0x06fa,
        char: 'З',
    },
    KeysymChar {
        keysym: 0x06fb,
        char: 'Ш',
    },
    KeysymChar {
        keysym: 0x06fc,
        char: 'Э',
    },
    KeysymChar {
        keysym: 0x06fd,
        char: 'Щ',
    },
    KeysymChar {
        keysym: 0x06fe,
        char: 'Ч',
    },
    KeysymChar {
        keysym: 0x06ff,
        char: 'Ъ',
    },
    KeysymChar {
        keysym: 0x07a1,
        char: 'Ά',
    },
    KeysymChar {
        keysym: 0x07a2,
        char: 'Έ',
    },
    KeysymChar {
        keysym: 0x07a3,
        char: 'Ή',
    },
    KeysymChar {
        keysym: 0x07a4,
        char: 'Ί',
    },
    KeysymChar {
        keysym: 0x07a5,
        char: 'Ϊ',
    },
    KeysymChar {
        keysym: 0x07a7,
        char: 'Ό',
    },
    KeysymChar {
        keysym: 0x07a8,
        char: 'Ύ',
    },
    KeysymChar {
        keysym: 0x07a9,
        char: 'Ϋ',
    },
    KeysymChar {
        keysym: 0x07ab,
        char: 'Ώ',
    },
    KeysymChar {
        keysym: 0x07ae,
        char: '΅',
    },
    KeysymChar {
        keysym: 0x07af,
        char: '―',
    },
    KeysymChar {
        keysym: 0x07b1,
        char: 'ά',
    },
    KeysymChar {
        keysym: 0x07b2,
        char: 'έ',
    },
    KeysymChar {
        keysym: 0x07b3,
        char: 'ή',
    },
    KeysymChar {
        keysym: 0x07b4,
        char: 'ί',
    },
    KeysymChar {
        keysym: 0x07b5,
        char: 'ϊ',
    },
    KeysymChar {
        keysym: 0x07b6,
        char: 'ΐ',
    },
    KeysymChar {
        keysym: 0x07b7,
        char: 'ό',
    },
    KeysymChar {
        keysym: 0x07b8,
        char: 'ύ',
    },
    KeysymChar {
        keysym: 0x07b9,
        char: 'ϋ',
    },
    KeysymChar {
        keysym: 0x07ba,
        char: 'ΰ',
    },
    KeysymChar {
        keysym: 0x07bb,
        char: 'ώ',
    },
    KeysymChar {
        keysym: 0x07c1,
        char: 'Α',
    },
    KeysymChar {
        keysym: 0x07c2,
        char: 'Β',
    },
    KeysymChar {
        keysym: 0x07c3,
        char: 'Γ',
    },
    KeysymChar {
        keysym: 0x07c4,
        char: 'Δ',
    },
    KeysymChar {
        keysym: 0x07c5,
        char: 'Ε',
    },
    KeysymChar {
        keysym: 0x07c6,
        char: 'Ζ',
    },
    KeysymChar {
        keysym: 0x07c7,
        char: 'Η',
    },
    KeysymChar {
        keysym: 0x07c8,
        char: 'Θ',
    },
    KeysymChar {
        keysym: 0x07c9,
        char: 'Ι',
    },
    KeysymChar {
        keysym: 0x07ca,
        char: 'Κ',
    },
    KeysymChar {
        keysym: 0x07cb,
        char: 'Λ',
    },
    KeysymChar {
        keysym: 0x07cc,
        char: 'Μ',
    },
    KeysymChar {
        keysym: 0x07cd,
        char: 'Ν',
    },
    KeysymChar {
        keysym: 0x07ce,
        char: 'Ξ',
    },
    KeysymChar {
        keysym: 0x07cf,
        char: 'Ο',
    },
    KeysymChar {
        keysym: 0x07d0,
        char: 'Π',
    },
    KeysymChar {
        keysym: 0x07d1,
        char: 'Ρ',
    },
    KeysymChar {
        keysym: 0x07d2,
        char: 'Σ',
    },
    KeysymChar {
        keysym: 0x07d4,
        char: 'Τ',
    },
    KeysymChar {
        keysym: 0x07d5,
        char: 'Υ',
    },
    KeysymChar {
        keysym: 0x07d6,
        char: 'Φ',
    },
    KeysymChar {
        keysym: 0x07d7,
        char: 'Χ',
    },
    KeysymChar {
        keysym: 0x07d8,
        char: 'Ψ',
    },
    KeysymChar {
        keysym: 0x07d9,
        char: 'Ω',
    },
    KeysymChar {
        keysym: 0x07e1,
        char: 'α',
    },
    KeysymChar {
        keysym: 0x07e2,
        char: 'β',
    },
    KeysymChar {
        keysym: 0x07e3,
        char: 'γ',
    },
    KeysymChar {
        keysym: 0x07e4,
        char: 'δ',
    },
    KeysymChar {
        keysym: 0x07e5,
        char: 'ε',
    },
    KeysymChar {
        keysym: 0x07e6,
        char: 'ζ',
    },
    KeysymChar {
        keysym: 0x07e7,
        char: 'η',
    },
    KeysymChar {
        keysym: 0x07e8,
        char: 'θ',
    },
    KeysymChar {
        keysym: 0x07e9,
        char: 'ι',
    },
    KeysymChar {
        keysym: 0x07ea,
        char: 'κ',
    },
    KeysymChar {
        keysym: 0x07eb,
        char: 'λ',
    },
    KeysymChar {
        keysym: 0x07ec,
        char: 'μ',
    },
    KeysymChar {
        keysym: 0x07ed,
        char: 'ν',
    },
    KeysymChar {
        keysym: 0x07ee,
        char: 'ξ',
    },
    KeysymChar {
        keysym: 0x07ef,
        char: 'ο',
    },
    KeysymChar {
        keysym: 0x07f0,
        char: 'π',
    },
    KeysymChar {
        keysym: 0x07f1,
        char: 'ρ',
    },
    KeysymChar {
        keysym: 0x07f2,
        char: 'σ',
    },
    KeysymChar {
        keysym: 0x07f3,
        char: 'ς',
    },
    KeysymChar {
        keysym: 0x07f4,
        char: 'τ',
    },
    KeysymChar {
        keysym: 0x07f5,
        char: 'υ',
    },
    KeysymChar {
        keysym: 0x07f6,
        char: 'φ',
    },
    KeysymChar {
        keysym: 0x07f7,
        char: 'χ',
    },
    KeysymChar {
        keysym: 0x07f8,
        char: 'ψ',
    },
    KeysymChar {
        keysym: 0x07f9,
        char: 'ω',
    },
    KeysymChar {
        keysym: 0x08a1,
        char: '⎷',
    },
    KeysymChar {
        keysym: 0x08a2,
        char: '┌',
    },
    KeysymChar {
        keysym: 0x08a3,
        char: '─',
    },
    KeysymChar {
        keysym: 0x08a4,
        char: '⌠',
    },
    KeysymChar {
        keysym: 0x08a5,
        char: '⌡',
    },
    KeysymChar {
        keysym: 0x08a6,
        char: '│',
    },
    KeysymChar {
        keysym: 0x08a7,
        char: '⎡',
    },
    KeysymChar {
        keysym: 0x08a8,
        char: '⎣',
    },
    KeysymChar {
        keysym: 0x08a9,
        char: '⎤',
    },
    KeysymChar {
        keysym: 0x08aa,
        char: '⎦',
    },
    KeysymChar {
        keysym: 0x08ab,
        char: '⎛',
    },
    KeysymChar {
        keysym: 0x08ac,
        char: '⎝',
    },
    KeysymChar {
        keysym: 0x08ad,
        char: '⎞',
    },
    KeysymChar {
        keysym: 0x08ae,
        char: '⎠',
    },
    KeysymChar {
        keysym: 0x08af,
        char: '⎨',
    },
    KeysymChar {
        keysym: 0x08b0,
        char: '⎬',
    },
    KeysymChar {
        keysym: 0x08bc,
        char: '≤',
    },
    KeysymChar {
        keysym: 0x08bd,
        char: '≠',
    },
    KeysymChar {
        keysym: 0x08be,
        char: '≥',
    },
    KeysymChar {
        keysym: 0x08bf,
        char: '∫',
    },
    KeysymChar {
        keysym: 0x08c0,
        char: '∴',
    },
    KeysymChar {
        keysym: 0x08c1,
        char: '∝',
    },
    KeysymChar {
        keysym: 0x08c2,
        char: '∞',
    },
    KeysymChar {
        keysym: 0x08c5,
        char: '∇',
    },
    KeysymChar {
        keysym: 0x08c8,
        char: '∼',
    },
    KeysymChar {
        keysym: 0x08c9,
        char: '≃',
    },
    KeysymChar {
        keysym: 0x08cd,
        char: '⇔',
    },
    KeysymChar {
        keysym: 0x08ce,
        char: '⇒',
    },
    KeysymChar {
        keysym: 0x08cf,
        char: '≡',
    },
    KeysymChar {
        keysym: 0x08d6,
        char: '√',
    },
    KeysymChar {
        keysym: 0x08da,
        char: '⊂',
    },
    KeysymChar {
        keysym: 0x08db,
        char: '⊃',
    },
    KeysymChar {
        keysym: 0x08dc,
        char: '∩',
    },
    KeysymChar {
        keysym: 0x08dd,
        char: '∪',
    },
    KeysymChar {
        keysym: 0x08de,
        char: '∧',
    },
    KeysymChar {
        keysym: 0x08df,
        char: '∨',
    },
    KeysymChar {
        keysym: 0x08ef,
        char: '∂',
    },
    KeysymChar {
        keysym: 0x08f6,
        char: 'ƒ',
    },
    KeysymChar {
        keysym: 0x08fb,
        char: '←',
    },
    KeysymChar {
        keysym: 0x08fc,
        char: '↑',
    },
    KeysymChar {
        keysym: 0x08fd,
        char: '→',
    },
    KeysymChar {
        keysym: 0x08fe,
        char: '↓',
    },
    KeysymChar {
        keysym: 0x09e0,
        char: '◆',
    },
    KeysymChar {
        keysym: 0x09e1,
        char: '▒',
    },
    KeysymChar {
        keysym: 0x09e2,
        char: '␉',
    },
    KeysymChar {
        keysym: 0x09e3,
        char: '␌',
    },
    KeysymChar {
        keysym: 0x09e4,
        char: '␍',
    },
    KeysymChar {
        keysym: 0x09e5,
        char: '␊',
    },
    KeysymChar {
        keysym: 0x09e8,
        char: '␤',
    },
    KeysymChar {
        keysym: 0x09e9,
        char: '␋',
    },
    KeysymChar {
        keysym: 0x09ea,
        char: '┘',
    },
    KeysymChar {
        keysym: 0x09eb,
        char: '┐',
    },
    KeysymChar {
        keysym: 0x09ec,
        char: '┌',
    },
    KeysymChar {
        keysym: 0x09ed,
        char: '└',
    },
    KeysymChar {
        keysym: 0x09ee,
        char: '┼',
    },
    KeysymChar {
        keysym: 0x09ef,
        char: '⎺',
    },
    KeysymChar {
        keysym: 0x09f0,
        char: '⎻',
    },
    KeysymChar {
        keysym: 0x09f1,
        char: '─',
    },
    KeysymChar {
        keysym: 0x09f2,
        char: '⎼',
    },
    KeysymChar {
        keysym: 0x09f3,
        char: '⎽',
    },
    KeysymChar {
        keysym: 0x09f4,
        char: '├',
    },
    KeysymChar {
        keysym: 0x09f5,
        char: '┤',
    },
    KeysymChar {
        keysym: 0x09f6,
        char: '┴',
    },
    KeysymChar {
        keysym: 0x09f7,
        char: '┬',
    },
    KeysymChar {
        keysym: 0x09f8,
        char: '│',
    },
    KeysymChar {
        keysym: 0x0aa1,
        char: '\u{2003}',
    },
    KeysymChar {
        keysym: 0x0aa2,
        char: '\u{2002}',
    },
    KeysymChar {
        keysym: 0x0aa3,
        char: '\u{2004}',
    },
    KeysymChar {
        keysym: 0x0aa4,
        char: '\u{2005}',
    },
    KeysymChar {
        keysym: 0x0aa5,
        char: '\u{2007}',
    },
    KeysymChar {
        keysym: 0x0aa6,
        char: '\u{2008}',
    },
    KeysymChar {
        keysym: 0x0aa7,
        char: '\u{2009}',
    },
    KeysymChar {
        keysym: 0x0aa8,
        char: '\u{200a}',
    },
    KeysymChar {
        keysym: 0x0aa9,
        char: '—',
    },
    KeysymChar {
        keysym: 0x0aaa,
        char: '–',
    },
    KeysymChar {
        keysym: 0x0aac,
        char: '␣',
    },
    KeysymChar {
        keysym: 0x0aae,
        char: '…',
    },
    KeysymChar {
        keysym: 0x0aaf,
        char: '‥',
    },
    KeysymChar {
        keysym: 0x0ab0,
        char: '⅓',
    },
    KeysymChar {
        keysym: 0x0ab1,
        char: '⅔',
    },
    KeysymChar {
        keysym: 0x0ab2,
        char: '⅕',
    },
    KeysymChar {
        keysym: 0x0ab3,
        char: '⅖',
    },
    KeysymChar {
        keysym: 0x0ab4,
        char: '⅗',
    },
    KeysymChar {
        keysym: 0x0ab5,
        char: '⅘',
    },
    KeysymChar {
        keysym: 0x0ab6,
        char: '⅙',
    },
    KeysymChar {
        keysym: 0x0ab7,
        char: '⅚',
    },
    KeysymChar {
        keysym: 0x0ab8,
        char: '℅',
    },
    KeysymChar {
        keysym: 0x0abb,
        char: '‒',
    },
    KeysymChar {
        keysym: 0x0abc,
        char: '⟨',
    },
    KeysymChar {
        keysym: 0x0abd,
        char: '.',
    },
    KeysymChar {
        keysym: 0x0abe,
        char: '⟩',
    },
    KeysymChar {
        keysym: 0x0ac3,
        char: '⅛',
    },
    KeysymChar {
        keysym: 0x0ac4,
        char: '⅜',
    },
    KeysymChar {
        keysym: 0x0ac5,
        char: '⅝',
    },
    KeysymChar {
        keysym: 0x0ac6,
        char: '⅞',
    },
    KeysymChar {
        keysym: 0x0ac9,
        char: '™',
    },
    KeysymChar {
        keysym: 0x0aca,
        char: '☓',
    },
    KeysymChar {
        keysym: 0x0acc,
        char: '◁',
    },
    KeysymChar {
        keysym: 0x0acd,
        char: '▷',
    },
    KeysymChar {
        keysym: 0x0ace,
        char: '○',
    },
    KeysymChar {
        keysym: 0x0acf,
        char: '▯',
    },
    KeysymChar {
        keysym: 0x0ad0,
        char: '‘',
    },
    KeysymChar {
        keysym: 0x0ad1,
        char: '’',
    },
    KeysymChar {
        keysym: 0x0ad2,
        char: '“',
    },
    KeysymChar {
        keysym: 0x0ad3,
        char: '”',
    },
    KeysymChar {
        keysym: 0x0ad4,
        char: '℞',
    },
    KeysymChar {
        keysym: 0x0ad5,
        char: '‰',
    },
    KeysymChar {
        keysym: 0x0ad6,
        char: '′',
    },
    KeysymChar {
        keysym: 0x0ad7,
        char: '″',
    },
    KeysymChar {
        keysym: 0x0ad9,
        char: '✝',
    },
    KeysymChar {
        keysym: 0x0adb,
        char: '▬',
    },
    KeysymChar {
        keysym: 0x0adc,
        char: '◀',
    },
    KeysymChar {
        keysym: 0x0add,
        char: '▶',
    },
    KeysymChar {
        keysym: 0x0ade,
        char: '●',
    },
    KeysymChar {
        keysym: 0x0adf,
        char: '▮',
    },
    KeysymChar {
        keysym: 0x0ae0,
        char: '◦',
    },
    KeysymChar {
        keysym: 0x0ae1,
        char: '▫',
    },
    KeysymChar {
        keysym: 0x0ae2,
        char: '▭',
    },
    KeysymChar {
        keysym: 0x0ae3,
        char: '△',
    },
    KeysymChar {
        keysym: 0x0ae4,
        char: '▽',
    },
    KeysymChar {
        keysym: 0x0ae5,
        char: '☆',
    },
    KeysymChar {
        keysym: 0x0ae6,
        char: '•',
    },
    KeysymChar {
        keysym: 0x0ae7,
        char: '▪',
    },
    KeysymChar {
        keysym: 0x0ae8,
        char: '▲',
    },
    KeysymChar {
        keysym: 0x0ae9,
        char: '▼',
    },
    KeysymChar {
        keysym: 0x0aea,
        char: '☜',
    },
    KeysymChar {
        keysym: 0x0aeb,
        char: '☞',
    },
    KeysymChar {
        keysym: 0x0aec,
        char: '♣',
    },
    KeysymChar {
        keysym: 0x0aed,
        char: '♦',
    },
    KeysymChar {
        keysym: 0x0aee,
        char: '♥',
    },
    KeysymChar {
        keysym: 0x0af0,
        char: '✠',
    },
    KeysymChar {
        keysym: 0x0af1,
        char: '†',
    },
    KeysymChar {
        keysym: 0x0af2,
        char: '‡',
    },
    KeysymChar {
        keysym: 0x0af3,
        char: '✓',
    },
    KeysymChar {
        keysym: 0x0af4,
        char: '✗',
    },
    KeysymChar {
        keysym: 0x0af5,
        char: '♯',
    },
    KeysymChar {
        keysym: 0x0af6,
        char: '♭',
    },
    KeysymChar {
        keysym: 0x0af7,
        char: '♂',
    },
    KeysymChar {
        keysym: 0x0af8,
        char: '♀',
    },
    KeysymChar {
        keysym: 0x0af9,
        char: '☎',
    },
    KeysymChar {
        keysym: 0x0afa,
        char: '⌕',
    },
    KeysymChar {
        keysym: 0x0afb,
        char: '℗',
    },
    KeysymChar {
        keysym: 0x0afc,
        char: '‸',
    },
    KeysymChar {
        keysym: 0x0afd,
        char: '‚',
    },
    KeysymChar {
        keysym: 0x0afe,
        char: '„',
    },
    KeysymChar {
        keysym: 0x0ba3,
        char: '<',
    },
    KeysymChar {
        keysym: 0x0ba6,
        char: '>',
    },
    KeysymChar {
        keysym: 0x0ba8,
        char: '∨',
    },
    KeysymChar {
        keysym: 0x0ba9,
        char: '∧',
    },
    KeysymChar {
        keysym: 0x0bc0,
        char: '¯',
    },
    KeysymChar {
        keysym: 0x0bc2,
        char: '⊤',
    },
    KeysymChar {
        keysym: 0x0bc3,
        char: '∩',
    },
    KeysymChar {
        keysym: 0x0bc4,
        char: '⌊',
    },
    KeysymChar {
        keysym: 0x0bc6,
        char: '_',
    },
    KeysymChar {
        keysym: 0x0bca,
        char: '∘',
    },
    KeysymChar {
        keysym: 0x0bcc,
        char: '⎕',
    },
    KeysymChar {
        keysym: 0x0bce,
        char: '⊥',
    },
    KeysymChar {
        keysym: 0x0bcf,
        char: '○',
    },
    KeysymChar {
        keysym: 0x0bd3,
        char: '⌈',
    },
    KeysymChar {
        keysym: 0x0bd6,
        char: '∪',
    },
    KeysymChar {
        keysym: 0x0bd8,
        char: '⊃',
    },
    KeysymChar {
        keysym: 0x0bda,
        char: '⊂',
    },
    KeysymChar {
        keysym: 0x0bdc,
        char: '⊣',
    },
    KeysymChar {
        keysym: 0x0bfc,
        char: '⊢',
    },
    KeysymChar {
        keysym: 0x0cdf,
        char: '‗',
    },
    KeysymChar {
        keysym: 0x0ce0,
        char: 'א',
    },
    KeysymChar {
        keysym: 0x0ce1,
        char: 'ב',
    },
    KeysymChar {
        keysym: 0x0ce2,
        char: 'ג',
    },
    KeysymChar {
        keysym: 0x0ce3,
        char: 'ד',
    },
    KeysymChar {
        keysym: 0x0ce4,
        char: 'ה',
    },
    KeysymChar {
        keysym: 0x0ce5,
        char: 'ו',
    },
    KeysymChar {
        keysym: 0x0ce6,
        char: 'ז',
    },
    KeysymChar {
        keysym: 0x0ce7,
        char: 'ח',
    },
    KeysymChar {
        keysym: 0x0ce8,
        char: 'ט',
    },
    KeysymChar {
        keysym: 0x0ce9,
        char: 'י',
    },
    KeysymChar {
        keysym: 0x0cea,
        char: 'ך',
    },
    KeysymChar {
        keysym: 0x0ceb,
        char: 'כ',
    },
    KeysymChar {
        keysym: 0x0cec,
        char: 'ל',
    },
    KeysymChar {
        keysym: 0x0ced,
        char: 'ם',
    },
    KeysymChar {
        keysym: 0x0cee,
        char: 'מ',
    },
    KeysymChar {
        keysym: 0x0cef,
        char: 'ן',
    },
    KeysymChar {
        keysym: 0x0cf0,
        char: 'נ',
    },
    KeysymChar {
        keysym: 0x0cf1,
        char: 'ס',
    },
    KeysymChar {
        keysym: 0x0cf2,
        char: 'ע',
    },
    KeysymChar {
        keysym: 0x0cf3,
        char: 'ף',
    },
    KeysymChar {
        keysym: 0x0cf4,
        char: 'פ',
    },
    KeysymChar {
        keysym: 0x0cf5,
        char: 'ץ',
    },
    KeysymChar {
        keysym: 0x0cf6,
        char: 'צ',
    },
    KeysymChar {
        keysym: 0x0cf7,
        char: 'ק',
    },
    KeysymChar {
        keysym: 0x0cf8,
        char: 'ר',
    },
    KeysymChar {
        keysym: 0x0cf9,
        char: 'ש',
    },
    KeysymChar {
        keysym: 0x0cfa,
        char: 'ת',
    },
    KeysymChar {
        keysym: 0x0da1,
        char: 'ก',
    },
    KeysymChar {
        keysym: 0x0da2,
        char: 'ข',
    },
    KeysymChar {
        keysym: 0x0da3,
        char: 'ฃ',
    },
    KeysymChar {
        keysym: 0x0da4,
        char: 'ค',
    },
    KeysymChar {
        keysym: 0x0da5,
        char: 'ฅ',
    },
    KeysymChar {
        keysym: 0x0da6,
        char: 'ฆ',
    },
    KeysymChar {
        keysym: 0x0da7,
        char: 'ง',
    },
    KeysymChar {
        keysym: 0x0da8,
        char: 'จ',
    },
    KeysymChar {
        keysym: 0x0da9,
        char: 'ฉ',
    },
    KeysymChar {
        keysym: 0x0daa,
        char: 'ช',
    },
    KeysymChar {
        keysym: 0x0dab,
        char: 'ซ',
    },
    KeysymChar {
        keysym: 0x0dac,
        char: 'ฌ',
    },
    KeysymChar {
        keysym: 0x0dad,
        char: 'ญ',
    },
    KeysymChar {
        keysym: 0x0dae,
        char: 'ฎ',
    },
    KeysymChar {
        keysym: 0x0daf,
        char: 'ฏ',
    },
    KeysymChar {
        keysym: 0x0db0,
        char: 'ฐ',
    },
    KeysymChar {
        keysym: 0x0db1,
        char: 'ฑ',
    },
    KeysymChar {
        keysym: 0x0db2,
        char: 'ฒ',
    },
    KeysymChar {
        keysym: 0x0db3,
        char: 'ณ',
    },
    KeysymChar {
        keysym: 0x0db4,
        char: 'ด',
    },
    KeysymChar {
        keysym: 0x0db5,
        char: 'ต',
    },
    KeysymChar {
        keysym: 0x0db6,
        char: 'ถ',
    },
    KeysymChar {
        keysym: 0x0db7,
        char: 'ท',
    },
    KeysymChar {
        keysym: 0x0db8,
        char: 'ธ',
    },
    KeysymChar {
        keysym: 0x0db9,
        char: 'น',
    },
    KeysymChar {
        keysym: 0x0dba,
        char: 'บ',
    },
    KeysymChar {
        keysym: 0x0dbb,
        char: 'ป',
    },
    KeysymChar {
        keysym: 0x0dbc,
        char: 'ผ',
    },
    KeysymChar {
        keysym: 0x0dbd,
        char: 'ฝ',
    },
    KeysymChar {
        keysym: 0x0dbe,
        char: 'พ',
    },
    KeysymChar {
        keysym: 0x0dbf,
        char: 'ฟ',
    },
    KeysymChar {
        keysym: 0x0dc0,
        char: 'ภ',
    },
    KeysymChar {
        keysym: 0x0dc1,
        char: 'ม',
    },
    KeysymChar {
        keysym: 0x0dc2,
        char: 'ย',
    },
    KeysymChar {
        keysym: 0x0dc3,
        char: 'ร',
    },
    KeysymChar {
        keysym: 0x0dc4,
        char: 'ฤ',
    },
    KeysymChar {
        keysym: 0x0dc5,
        char: 'ล',
    },
    KeysymChar {
        keysym: 0x0dc6,
        char: 'ฦ',
    },
    KeysymChar {
        keysym: 0x0dc7,
        char: 'ว',
    },
    KeysymChar {
        keysym: 0x0dc8,
        char: 'ศ',
    },
    KeysymChar {
        keysym: 0x0dc9,
        char: 'ษ',
    },
    KeysymChar {
        keysym: 0x0dca,
        char: 'ส',
    },
    KeysymChar {
        keysym: 0x0dcb,
        char: 'ห',
    },
    KeysymChar {
        keysym: 0x0dcc,
        char: 'ฬ',
    },
    KeysymChar {
        keysym: 0x0dcd,
        char: 'อ',
    },
    KeysymChar {
        keysym: 0x0dce,
        char: 'ฮ',
    },
    KeysymChar {
        keysym: 0x0dcf,
        char: 'ฯ',
    },
    KeysymChar {
        keysym: 0x0dd0,
        char: 'ะ',
    },
    KeysymChar {
        keysym: 0x0dd1,
        char: '\u{e31}',
    },
    KeysymChar {
        keysym: 0x0dd2,
        char: 'า',
    },
    KeysymChar {
        keysym: 0x0dd3,
        char: 'ำ',
    },
    KeysymChar {
        keysym: 0x0dd4,
        char: '\u{e34}',
    },
    KeysymChar {
        keysym: 0x0dd5,
        char: '\u{e35}',
    },
    KeysymChar {
        keysym: 0x0dd6,
        char: '\u{e36}',
    },
    KeysymChar {
        keysym: 0x0dd7,
        char: '\u{e37}',
    },
    KeysymChar {
        keysym: 0x0dd8,
        char: '\u{e38}',
    },
    KeysymChar {
        keysym: 0x0dd9,
        char: '\u{e39}',
    },
    KeysymChar {
        keysym: 0x0dda,
        char: '\u{e3a}',
    },
    KeysymChar {
        keysym: 0x0dde,
        char: '\u{e3e}',
    },
    KeysymChar {
        keysym: 0x0ddf,
        char: '฿',
    },
    KeysymChar {
        keysym: 0x0de0,
        char: 'เ',
    },
    KeysymChar {
        keysym: 0x0de1,
        char: 'แ',
    },
    KeysymChar {
        keysym: 0x0de2,
        char: 'โ',
    },
    KeysymChar {
        keysym: 0x0de3,
        char: 'ใ',
    },
    KeysymChar {
        keysym: 0x0de4,
        char: 'ไ',
    },
    KeysymChar {
        keysym: 0x0de5,
        char: 'ๅ',
    },
    KeysymChar {
        keysym: 0x0de6,
        char: 'ๆ',
    },
    KeysymChar {
        keysym: 0x0de7,
        char: '\u{e47}',
    },
    KeysymChar {
        keysym: 0x0de8,
        char: '\u{e48}',
    },
    KeysymChar {
        keysym: 0x0de9,
        char: '\u{e49}',
    },
    KeysymChar {
        keysym: 0x0dea,
        char: '\u{e4a}',
    },
    KeysymChar {
        keysym: 0x0deb,
        char: '\u{e4b}',
    },
    KeysymChar {
        keysym: 0x0dec,
        char: '\u{e4c}',
    },
    KeysymChar {
        keysym: 0x0ded,
        char: '\u{e4d}',
    },
    KeysymChar {
        keysym: 0x0df0,
        char: '๐',
    },
    KeysymChar {
        keysym: 0x0df1,
        char: '๑',
    },
    KeysymChar {
        keysym: 0x0df2,
        char: '๒',
    },
    KeysymChar {
        keysym: 0x0df3,
        char: '๓',
    },
    KeysymChar {
        keysym: 0x0df4,
        char: '๔',
    },
    KeysymChar {
        keysym: 0x0df5,
        char: '๕',
    },
    KeysymChar {
        keysym: 0x0df6,
        char: '๖',
    },
    KeysymChar {
        keysym: 0x0df7,
        char: '๗',
    },
    KeysymChar {
        keysym: 0x0df8,
        char: '๘',
    },
    KeysymChar {
        keysym: 0x0df9,
        char: '๙',
    },
    KeysymChar {
        keysym: 0x0ea1,
        char: 'ㄱ',
    },
    KeysymChar {
        keysym: 0x0ea2,
        char: 'ㄲ',
    },
    KeysymChar {
        keysym: 0x0ea3,
        char: 'ㄳ',
    },
    KeysymChar {
        keysym: 0x0ea4,
        char: 'ㄴ',
    },
    KeysymChar {
        keysym: 0x0ea5,
        char: 'ㄵ',
    },
    KeysymChar {
        keysym: 0x0ea6,
        char: 'ㄶ',
    },
    KeysymChar {
        keysym: 0x0ea7,
        char: 'ㄷ',
    },
    KeysymChar {
        keysym: 0x0ea8,
        char: 'ㄸ',
    },
    KeysymChar {
        keysym: 0x0ea9,
        char: 'ㄹ',
    },
    KeysymChar {
        keysym: 0x0eaa,
        char: 'ㄺ',
    },
    KeysymChar {
        keysym: 0x0eab,
        char: 'ㄻ',
    },
    KeysymChar {
        keysym: 0x0eac,
        char: 'ㄼ',
    },
    KeysymChar {
        keysym: 0x0ead,
        char: 'ㄽ',
    },
    KeysymChar {
        keysym: 0x0eae,
        char: 'ㄾ',
    },
    KeysymChar {
        keysym: 0x0eaf,
        char: 'ㄿ',
    },
    KeysymChar {
        keysym: 0x0eb0,
        char: 'ㅀ',
    },
    KeysymChar {
        keysym: 0x0eb1,
        char: 'ㅁ',
    },
    KeysymChar {
        keysym: 0x0eb2,
        char: 'ㅂ',
    },
    KeysymChar {
        keysym: 0x0eb3,
        char: 'ㅃ',
    },
    KeysymChar {
        keysym: 0x0eb4,
        char: 'ㅄ',
    },
    KeysymChar {
        keysym: 0x0eb5,
        char: 'ㅅ',
    },
    KeysymChar {
        keysym: 0x0eb6,
        char: 'ㅆ',
    },
    KeysymChar {
        keysym: 0x0eb7,
        char: 'ㅇ',
    },
    KeysymChar {
        keysym: 0x0eb8,
        char: 'ㅈ',
    },
    KeysymChar {
        keysym: 0x0eb9,
        char: 'ㅉ',
    },
    KeysymChar {
        keysym: 0x0eba,
        char: 'ㅊ',
    },
    KeysymChar {
        keysym: 0x0ebb,
        char: 'ㅋ',
    },
    KeysymChar {
        keysym: 0x0ebc,
        char: 'ㅌ',
    },
    KeysymChar {
        keysym: 0x0ebd,
        char: 'ㅍ',
    },
    KeysymChar {
        keysym: 0x0ebe,
        char: 'ㅎ',
    },
    KeysymChar {
        keysym: 0x0ebf,
        char: 'ㅏ',
    },
    KeysymChar {
        keysym: 0x0ec0,
        char: 'ㅐ',
    },
    KeysymChar {
        keysym: 0x0ec1,
        char: 'ㅑ',
    },
    KeysymChar {
        keysym: 0x0ec2,
        char: 'ㅒ',
    },
    KeysymChar {
        keysym: 0x0ec3,
        char: 'ㅓ',
    },
    KeysymChar {
        keysym: 0x0ec4,
        char: 'ㅔ',
    },
    KeysymChar {
        keysym: 0x0ec5,
        char: 'ㅕ',
    },
    KeysymChar {
        keysym: 0x0ec6,
        char: 'ㅖ',
    },
    KeysymChar {
        keysym: 0x0ec7,
        char: 'ㅗ',
    },
    KeysymChar {
        keysym: 0x0ec8,
        char: 'ㅘ',
    },
    KeysymChar {
        keysym: 0x0ec9,
        char: 'ㅙ',
    },
    KeysymChar {
        keysym: 0x0eca,
        char: 'ㅚ',
    },
    KeysymChar {
        keysym: 0x0ecb,
        char: 'ㅛ',
    },
    KeysymChar {
        keysym: 0x0ecc,
        char: 'ㅜ',
    },
    KeysymChar {
        keysym: 0x0ecd,
        char: 'ㅝ',
    },
    KeysymChar {
        keysym: 0x0ece,
        char: 'ㅞ',
    },
    KeysymChar {
        keysym: 0x0ecf,
        char: 'ㅟ',
    },
    KeysymChar {
        keysym: 0x0ed0,
        char: 'ㅠ',
    },
    KeysymChar {
        keysym: 0x0ed1,
        char: 'ㅡ',
    },
    KeysymChar {
        keysym: 0x0ed2,
        char: 'ㅢ',
    },
    KeysymChar {
        keysym: 0x0ed3,
        char: 'ㅣ',
    },
    KeysymChar {
        keysym: 0x0ed4,
        char: 'ᆨ',
    },
    KeysymChar {
        keysym: 0x0ed5,
        char: 'ᆩ',
    },
    KeysymChar {
        keysym: 0x0ed6,
        char: 'ᆪ',
    },
    KeysymChar {
        keysym: 0x0ed7,
        char: 'ᆫ',
    },
    KeysymChar {
        keysym: 0x0ed8,
        char: 'ᆬ',
    },
    KeysymChar {
        keysym: 0x0ed9,
        char: 'ᆭ',
    },
    KeysymChar {
        keysym: 0x0eda,
        char: 'ᆮ',
    },
    KeysymChar {
        keysym: 0x0edb,
        char: 'ᆯ',
    },
    KeysymChar {
        keysym: 0x0edc,
        char: 'ᆰ',
    },
    KeysymChar {
        keysym: 0x0edd,
        char: 'ᆱ',
    },
    KeysymChar {
        keysym: 0x0ede,
        char: 'ᆲ',
    },
    KeysymChar {
        keysym: 0x0edf,
        char: 'ᆳ',
    },
    KeysymChar {
        keysym: 0x0ee0,
        char: 'ᆴ',
    },
    KeysymChar {
        keysym: 0x0ee1,
        char: 'ᆵ',
    },
    KeysymChar {
        keysym: 0x0ee2,
        char: 'ᆶ',
    },
    KeysymChar {
        keysym: 0x0ee3,
        char: 'ᆷ',
    },
    KeysymChar {
        keysym: 0x0ee4,
        char: 'ᆸ',
    },
    KeysymChar {
        keysym: 0x0ee5,
        char: 'ᆹ',
    },
    KeysymChar {
        keysym: 0x0ee6,
        char: 'ᆺ',
    },
    KeysymChar {
        keysym: 0x0ee7,
        char: 'ᆻ',
    },
    KeysymChar {
        keysym: 0x0ee8,
        char: 'ᆼ',
    },
    KeysymChar {
        keysym: 0x0ee9,
        char: 'ᆽ',
    },
    KeysymChar {
        keysym: 0x0eea,
        char: 'ᆾ',
    },
    KeysymChar {
        keysym: 0x0eeb,
        char: 'ᆿ',
    },
    KeysymChar {
        keysym: 0x0eec,
        char: 'ᇀ',
    },
    KeysymChar {
        keysym: 0x0eed,
        char: 'ᇁ',
    },
    KeysymChar {
        keysym: 0x0eee,
        char: 'ᇂ',
    },
    KeysymChar {
        keysym: 0x0eef,
        char: 'ㅭ',
    },
    KeysymChar {
        keysym: 0x0ef0,
        char: 'ㅱ',
    },
    KeysymChar {
        keysym: 0x0ef1,
        char: 'ㅸ',
    },
    KeysymChar {
        keysym: 0x0ef2,
        char: 'ㅿ',
    },
    KeysymChar {
        keysym: 0x0ef3,
        char: 'ㆁ',
    },
    KeysymChar {
        keysym: 0x0ef4,
        char: 'ㆄ',
    },
    KeysymChar {
        keysym: 0x0ef5,
        char: 'ㆆ',
    },
    KeysymChar {
        keysym: 0x0ef6,
        char: 'ㆍ',
    },
    KeysymChar {
        keysym: 0x0ef7,
        char: 'ㆎ',
    },
    KeysymChar {
        keysym: 0x0ef8,
        char: 'ᇫ',
    },
    KeysymChar {
        keysym: 0x0ef9,
        char: 'ᇰ',
    },
    KeysymChar {
        keysym: 0x0efa,
        char: 'ᇹ',
    },
    KeysymChar {
        keysym: 0x0eff,
        char: '₩',
    },
    KeysymChar {
        keysym: 0x13bc,
        char: 'Œ',
    },
    KeysymChar {
        keysym: 0x13bd,
        char: 'œ',
    },
    KeysymChar {
        keysym: 0x13be,
        char: 'Ÿ',
    },
    KeysymChar {
        keysym: 0x20ac,
        char: '€',
    },
    KeysymChar {
        keysym: 0xff08,
        char: '\u{8}',
    },
    KeysymChar {
        keysym: 0xff09,
        char: '\t',
    },
    KeysymChar {
        keysym: 0xff0a,
        char: '\n',
    },
    KeysymChar {
        keysym: 0xff0b,
        char: '\u{b}',
    },
    KeysymChar {
        keysym: 0xff0d,
        char: '\r',
    },
    KeysymChar {
        keysym: 0xff1b,
        char: '\u{1b}',
    },
    KeysymChar {
        keysym: 0xff80,
        char: ' ',
    },
    KeysymChar {
        keysym: 0xff89,
        char: '\t',
    },
    KeysymChar {
        keysym: 0xff8d,
        char: '\r',
    },
    KeysymChar {
        keysym: 0xffaa,
        char: '*',
    },
    KeysymChar {
        keysym: 0xffab,
        char: '+',
    },
    KeysymChar {
        keysym: 0xffac,
        char: ',',
    },
    KeysymChar {
        keysym: 0xffad,
        char: '-',
    },
    KeysymChar {
        keysym: 0xffae,
        char: '.',
    },
    KeysymChar {
        keysym: 0xffaf,
        char: '/',
    },
    KeysymChar {
        keysym: 0xffb0,
        char: '0',
    },
    KeysymChar {
        keysym: 0xffb1,
        char: '1',
    },
    KeysymChar {
        keysym: 0xffb2,
        char: '2',
    },
    KeysymChar {
        keysym: 0xffb3,
        char: '3',
    },
    KeysymChar {
        keysym: 0xffb4,
        char: '4',
    },
    KeysymChar {
        keysym: 0xffb5,
        char: '5',
    },
    KeysymChar {
        keysym: 0xffb6,
        char: '6',
    },
    KeysymChar {
        keysym: 0xffb7,
        char: '7',
    },
    KeysymChar {
        keysym: 0xffb8,
        char: '8',
    },
    KeysymChar {
        keysym: 0xffb9,
        char: '9',
    },
    KeysymChar {
        keysym: 0xffbd,
        char: '=',
    },
    KeysymChar {
        keysym: 0xffff,
        char: '\u{7f}',
    },
];
pub(super) static KEYSYM32_TO_CHAR: &[Keysym32Char] = &[
];

pub(super) static CHAR_TO_BESPOKE_IDX: PhfMap<char, u16> = PhfMap {
    key: 12913932095322966823,
    disps: &[(0, 201), (0, 13), (0, 0), (0, 46), (0, 96), (0, 0), (0, 56), (0, 30), (0, 7), (0, 205), (0, 116), (0, 4), (0, 47), (0, 3), (0, 7), (0, 0), (0, 268), (0, 100), (0, 0), (0, 34), (0, 306), (0, 256), (0, 21), (0, 176), (0, 17), (0, 0), (0, 10), (0, 0), (0, 4), (0, 0), (0, 1), (0, 806), (0, 1), (0, 0), (0, 93), (0, 1), (0, 0), (0, 0), (0, 0), (0, 1), (0, 28), (0, 0), (0, 0), (0, 0), (0, 0), (0, 96), (0, 1), (0, 1), (0, 43), (0, 36), (0, 767), (0, 7), (0, 52), (0, 0), (0, 3), (0, 24), (0, 8), (0, 62), (0, 0), (0, 11), (0, 73), (0, 0), (0, 2), (0, 167), (0, 0), (0, 0), (0, 71), (0, 5), (0, 34), (0, 92), (0, 16), (0, 0), (0, 62), (0, 175), (0, 1), (0, 110), (0, 116), (0, 0), (0, 58), (0, 2), (0, 2), (0, 4), (0, 11), (0, 7), (0, 0), (0, 4), (0, 31), (0, 3), (0, 176), (0, 34), (0, 25), (0, 0), (0, 27), (0, 10), (0, 469), (0, 19), (0, 8), (0, 792), (0, 5), (0, 35), (0, 10), (0, 4), (0, 229), (0, 132), (0, 15), (0, 0), (0, 85), (0, 151), (0, 16), (0, 0), (0, 1), (0, 2), (0, 0), (0, 1), (0, 12), (0, 4), (0, 47), (0, 6), (0, 22), (0, 8), (0, 211), (0, 0), (0, 12), (0, 0), (0, 0), (0, 50), (0, 0), (0, 6), (0, 3), (0, 5), (0, 7), (0, 221), (0, 0), (0, 98), (0, 143), (0, 37), (0, 46), (0, 377), (0, 15), (0, 15), (0, 30), (0, 9), (0, 17), (0, 43), (0, 30), (0, 415), (0, 4), (0, 4), (0, 18), (0, 0), (0, 5), (0, 99), (0, 445), (0, 152), (0, 33), (0, 162), (0, 255), (0, 578), (0, 218), (0, 163), (1, 459), (0, 7), (0, 0), (2, 192), (0, 6), (0, 67), (0, 56), (0, 22), (0, 6), (0, 428), (0, 16), (0, 5), (0, 146), (0, 731), (0, 23), (0, 6), (0, 10), (0, 414), (0, 107), (0, 32), (0, 145), (0, 90), (0, 237), (0, 7), (0, 4), (0, 151), (0, 7), (0, 507), (0, 432), (0, 36), (0, 75), (0, 88), (0, 51), (0, 326), (0, 17), (0, 258), (0, 3), (0, 238), (0, 0), (0, 4), (0, 0), (0, 161), (0, 1), (0, 37), (0, 4), (0, 120), (0, 4), (0, 8), (0, 3), (0, 0), (0, 698), (0, 7), (0, 0), (0, 17), (0, 164), (0, 48), (1, 258), (0, 2), (0, 220), (0, 553), (0, 857), (0, 177), (0, 9), (1, 66), (4, 46), (0, 240), (1, 505), (0, 46), (0, 876), (0, 1), (0, 14), (2, 421), (0, 320), (2, 426), (1, 69), (0, 33), (0, 17), (2, 108), (0, 2), (0, 1), (0, 14), (1, 634), (0, 10), (2, 579), (0, 266), (0, 131), (0, 8), (2, 357), (0, 175), (1, 87), (0, 184), (0, 604), (0, 65), (4, 712), (0, 1), (0, 12)],
    map: &[294, 645, 222, 648, 1008, 847, 872, 397, 435, 355, 793, 554, 155, 715, 553, 908, 111, 932, 16, 102, 318, 742, 965, 221, 838, 685, 65, 332, 393, 68, 124, 531, 94, 233, 356, 555, 885, 659, 491, 911, 772, 522, 218, 264, 45, 759, 346, 278, 486, 693, 171, 359, 161, 571, 686, 949, 33, 898, 616, 121, 377, 985, 85, 388, 732, 675, 410, 810, 944, 366, 295, 839, 36, 122, 195, 370, 672, 277, 244, 562, 430, 711, 62, 891, 269, 961, 604, 494, 13, 863, 509, 971, 7, 963, 591, 77, 188, 37, 878, 337, 215, 372, 303, 620, 869, 120, 536, 273, 411, 456, 520, 201, 626, 312, 1001, 25, 146, 168, 480, 725, 431, 234, 895, 30, 309, 568, 214, 444, 688, 251, 506, 929, 781, 297, 440, 516, 615, 821, 143, 157, 39, 792, 165, 138, 655, 492, 54, 973, 112, 427, 856, 928, 4, 748, 981, 257, 185, 580, 958, 225, 848, 910, 131, 583, 403, 660, 767, 769, 866, 1191, 306, 253, 933, 538, 287, 877, 727, 133, 6, 887, 710, 22, 394, 706, 476, 446, 497, 760, 656, 73, 246, 459, 327, 151, 353, 285, 844, 920, 378, 699, 926, 302, 990, 69, 53, 290, 968, 342, 103, 51, 950, 565, 762, 489, 177, 90, 903, 245, 840, 415, 457, 904, 364, 176, 436, 135, 331, 252, 883, 130, 460, 91, 617, 861, 270, 261, 534, 640, 153, 14, 127, 100, 213, 592, 897, 1009, 730, 836, 982, 280, 336, 471, 255, 391, 93, 738, 547, 1187, 964, 321, 691, 676, 613, 462, 507, 375, 805, 690, 254, 345, 481, 803, 169, 334, 1002, 194, 351, 412, 518, 606, 265, 734, 747, 587, 474, 853, 31, 901, 664, 647, 523, 202, 144, 807, 578, 896, 408, 830, 5, 60, 938, 875, 652, 363, 822, 947, 74, 959, 995, 212, 684, 247, 226, 186, 139, 118, 757, 801, 601, 239, 590, 453, 979, 724, 941, 35, 293, 1000, 540, 341, 668, 163, 387, 867, 625, 549, 262, 75, 360, 939, 205, 395, 204, 357, 667, 28, 182, 845, 749, 893, 291, 529, 400, 975, 997, 236, 428, 794, 23, 927, 819, 382, 519, 52, 2, 597, 487, 701, 46, 598, 141, 210, 140, 425, 708, 243, 92, 238, 418, 298, 735, 1007, 862, 649, 1006, 477, 326, 918, 942, 203, 736, 787, 208, 276, 864, 304, 886, 174, 745, 931, 545, 632, 1003, 160, 88, 837, 101, 479, 392, 66, 1004, 20, 919, 768, 842, 614, 512, 44, 813, 654, 876, 473, 679, 376, 765, 700, 665, 71, 558, 881, 815, 83, 948, 63, 465, 707, 84, 671, 588, 575, 292, 191, 119, 409, 43, 501, 454, 966, 992, 661, 756, 996, 854, 367, 511, 921, 286, 335, 496, 983, 330, 59, 12, 687, 696, 750, 859, 11, 644, 611, 978, 310, 389, 242, 733, 902, 766, 129, 504, 371, 826, 849, 761, 416, 57, 61, 855, 924, 980, 802, 344, 940, 495, 662, 566, 299, 317, 719, 219, 567, 29, 785, 97, 467, 442, 339, 288, 851, 754, 651, 406, 585, 514, 884, 362, 834, 521, 56, 936, 828, 755, 873, 183, 451, 913, 599, 608, 429, 348, 957, 258, 653, 894, 296, 184, 582, 417, 72, 956, 977, 704, 358, 797, 109, 142, 237, 763, 914, 909, 220, 505, 741, 865, 999, 274, 546, 175, 953, 421, 698, 272, 180, 21, 340, 89, 581, 843, 989, 26, 814, 513, 689, 398, 646, 786, 589, 229, 816, 673, 107, 50, 595, 972, 697, 746, 156, 17, 426, 490, 305, 423, 488, 271, 969, 916, 211, 134, 500, 200, 249, 543, 882, 325, 993, 860, 631, 561, 1188, 628, 988, 596, 374, 694, 152, 612, 532, 998, 81, 40, 338, 744, 600, 172, 235, 483, 49, 770, 572, 899, 18, 275, 559, 722, 986, 385, 301, 198, 984, 390, 974, 917, 605, 674, 307, 8, 503, 240, 147, 811, 482, 650, 663, 945, 116, 720, 82, 260, 874, 731, 752, 586, 962, 852, 284, 117, 347, 784, 437, 263, 475, 279, 857, 621, 329, 99, 870, 712, 407, 167, 533, 552, 791, 386, 641, 829, 937, 78, 535, 743, 728, 1397, 485, 113, 250, 799, 683, 900, 58, 556, 702, 930, 316, 915, 758, 42, 771, 189, 739, 315, 154, 526, 820, 954, 349, 414, 343, 181, 1005, 717, 493, 569, 892, 27, 399, 132, 86, 55, 832, 136, 368, 10, 125, 404, 753, 517, 817, 383, 610, 912, 95, 241, 1, 594, 70, 934, 657, 1010, 224, 227, 888, 381, 424, 970, 560, 783, 705, 955, 3, 564, 925, 232, 1189, 47, 629, 557, 498, 104, 448, 681, 283, 256, 946, 419, 379, 173, 267, 764, 158, 630, 951, 721, 19, 80, 223, 835, 231, 311, 841, 268, 178, 987, 703, 994, 905, 871, 812, 678, 670, 87, 923, 484, 313, 579, 148, 713, 64, 259, 642, 15, 682, 544, 34, 463, 190, 209, 1195, 41, 228, 434, 528, 695, 677, 527, 880, 551, 373, 574, 452, 38, 413, 627, 858, 432, 149, 539, 779, 609, 943, 692, 502, 206, 170, 508, 79, 729, 550, 833, 32, 300, 576, 319, 324, 192, 207, 433, 624, 328, 123, 809, 196, 108, 570, 361, 524, 510, 525, 669, 114, 469, 824, 320, 906, 718, 890, 537, 991, 960, 98, 603, 573, 967, 187, 584, 396, 369, 850, 308, 384, 197, 115, 405, 282, 216, 530, 1190, 935, 868, 450, 199, 230, 542, 266, 439, 889, 145, 164, 643, 680, 289, 879, 976, 96, 846, 607, 458, 740, 401, 48, 795, 137, 162, 217, 248, 563, 76, 515, 623, 420, 658, 380, 402, 499, 323, 922, 350, 907, 709, 952, 179, 751, 541, 106, 24, 593, 105, 314],
    _phantom: core::marker::PhantomData,
};

pub(super) static DATAS: &[KeysymData] = &[
    KeysymData {
        keysym_or_definitive_idx: 0x00000000,
        name_start: 0,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000020,
        name_start: 8,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000021,
        name_start: 13,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000022,
        name_start: 19,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000023,
        name_start: 27,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000024,
        name_start: 37,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000025,
        name_start: 43,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000026,
        name_start: 50,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000027,
        name_start: 59,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000008,
        name_start: 69,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000028,
        name_start: 79,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000029,
        name_start: 88,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002a,
        name_start: 98,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002b,
        name_start: 106,
        name_len: 4,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002c,
        name_start: 110,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002d,
        name_start: 115,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002e,
        name_start: 120,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002f,
        name_start: 126,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000030,
        name_start: 131,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000031,
        name_start: 132,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000032,
        name_start: 133,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000033,
        name_start: 134,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000034,
        name_start: 135,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000035,
        name_start: 136,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000036,
        name_start: 137,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000037,
        name_start: 138,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000038,
        name_start: 139,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000039,
        name_start: 140,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003a,
        name_start: 141,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003b,
        name_start: 146,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003c,
        name_start: 155,
        name_len: 4,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003d,
        name_start: 159,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003e,
        name_start: 164,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003f,
        name_start: 171,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000040,
        name_start: 179,
        name_len: 2,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000041,
        name_start: 181,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000042,
        name_start: 182,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000043,
        name_start: 183,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000044,
        name_start: 184,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000045,
        name_start: 185,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000046,
        name_start: 186,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000047,
        name_start: 187,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000048,
        name_start: 188,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000049,
        name_start: 189,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004a,
        name_start: 190,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004b,
        name_start: 191,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004c,
        name_start: 192,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004d,
        name_start: 193,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004e,
        name_start: 194,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004f,
        name_start: 195,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000050,
        name_start: 196,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000051,
        name_start: 197,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000052,
        name_start: 198,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000053,
        name_start: 199,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000054,
        name_start: 200,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000055,
        name_start: 201,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000056,
        name_start: 202,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000057,
        name_start: 203,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000058,
        name_start: 204,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000059,
        name_start: 205,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005a,
        name_start: 206,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005b,
        name_start: 207,
        name_len: 11,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005c,
        name_start: 218,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005d,
        name_start: 227,
        name_len: 12,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005e,
        name_start: 239,
        name_len: 11,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005f,
        name_start: 250,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000060,
        name_start: 260,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000042,
        name_start: 265,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000061,
        name_start: 274,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000062,
        name_start: 275,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000063,
        name_start: 276,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000064,
        name_start: 277,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000065,
        name_start: 278,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000066,
        name_start: 279,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000067,
        name_start: 280,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000068,
        name_start: 281,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000069,
        name_start: 282,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006a,
        name_start: 283,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006b,
        name_start: 284,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006c,
        name_start: 285,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006d,
        name_start: 286,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006e,
        name_start: 287,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006f,
        name_start: 288,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000070,
        name_start: 289,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000071,
        name_start: 290,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000072,
        name_start: 291,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000073,
        name_start: 292,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000074,
        name_start: 293,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000075,
        name_start: 294,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000076,
        name_start: 295,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000077,
        name_start: 296,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000078,
        name_start: 297,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000079,
        name_start: 298,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007a,
        name_start: 299,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007b,
        name_start: 300,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007c,
        name_start: 309,
        name_len: 3,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007d,
        name_start: 312,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007e,
        name_start: 322,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a0,
        name_start: 332,
        name_len: 12,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a1,
        name_start: 344,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a2,
        name_start: 354,
        name_len: 4,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a3,
        name_start: 358,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a4,
        name_start: 366,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a5,
        name_start: 374,
        name_len: 3,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a6,
        name_start: 377,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a7,
        name_start: 386,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a8,
        name_start: 393,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a9,
        name_start: 402,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000aa,
        name_start: 411,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ab,
        name_start: 422,
        name_len: 13,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006d,
        name_start: 435,
        name_len: 13,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ac,
        name_start: 448,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ad,
        name_start: 455,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ae,
        name_start: 461,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000af,
        name_start: 471,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b0,
        name_start: 477,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b1,
        name_start: 483,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b2,
        name_start: 492,
        name_len: 11,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b3,
        name_start: 503,
        name_len: 13,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b4,
        name_start: 516,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b5,
        name_start: 521,
        name_len: 2,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b6,
        name_start: 523,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b7,
        name_start: 532,
        name_len: 14,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b8,
        name_start: 546,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b9,
        name_start: 553,
        name_len: 11,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ba,
        name_start: 564,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007d,
        name_start: 573,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000bb,
        name_start: 585,
        name_len: 14,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007f,
        name_start: 599,
        name_len: 14,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000bc,
        name_start: 613,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000bd,
        name_start: 623,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000be,
        name_start: 630,
        name_len: 13,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000bf,
        name_start: 643,
        name_len: 12,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c0,
        name_start: 655,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c1,
        name_start: 661,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c2,
        name_start: 667,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c3,
        name_start: 678,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c4,
        name_start: 684,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c5,
        name_start: 694,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c6,
        name_start: 699,
        name_len: 2,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c7,
        name_start: 701,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c8,
        name_start: 709,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c9,
        name_start: 715,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ca,
        name_start: 721,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000cb,
        name_start: 732,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000cc,
        name_start: 742,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000cd,
        name_start: 748,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ce,
        name_start: 754,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000cf,
        name_start: 765,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d0,
        name_start: 775,
        name_len: 3,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000095,
        name_start: 778,
        name_len: 3,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d1,
        name_start: 781,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d2,
        name_start: 787,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d3,
        name_start: 793,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d4,
        name_start: 799,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d5,
        name_start: 810,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d6,
        name_start: 816,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d7,
        name_start: 826,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d8,
        name_start: 834,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000009e,
        name_start: 840,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d9,
        name_start: 848,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000da,
        name_start: 854,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000db,
        name_start: 860,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000dc,
        name_start: 871,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000dd,
        name_start: 881,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000de,
        name_start: 887,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a5,
        name_start: 892,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000df,
        name_start: 897,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e0,
        name_start: 903,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e1,
        name_start: 909,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e2,
        name_start: 915,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e3,
        name_start: 926,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e4,
        name_start: 932,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e5,
        name_start: 942,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e6,
        name_start: 947,
        name_len: 2,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e7,
        name_start: 949,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e8,
        name_start: 957,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e9,
        name_start: 963,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ea,
        name_start: 969,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000eb,
        name_start: 980,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ec,
        name_start: 990,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ed,
        name_start: 996,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ee,
        name_start: 1002,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ef,
        name_start: 1013,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f0,
        name_start: 1023,
        name_len: 3,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f1,
        name_start: 1026,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f2,
        name_start: 1032,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f3,
        name_start: 1038,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f4,
        name_start: 1044,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f5,
        name_start: 1055,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f6,
        name_start: 1061,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f7,
        name_start: 1071,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f8,
        name_start: 1079,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c0,
        name_start: 1085,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f9,
        name_start: 1093,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fa,
        name_start: 1099,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fb,
        name_start: 1105,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fc,
        name_start: 1116,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fd,
        name_start: 1126,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fe,
        name_start: 1132,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ff,
        name_start: 1137,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a1,
        name_start: 1147,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a2,
        name_start: 1154,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a3,
        name_start: 1159,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a5,
        name_start: 1166,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a6,
        name_start: 1172,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a9,
        name_start: 1178,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001aa,
        name_start: 1184,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ab,
        name_start: 1192,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ac,
        name_start: 1198,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ae,
        name_start: 1204,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001af,
        name_start: 1210,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b1,
        name_start: 1219,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b2,
        name_start: 1226,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b3,
        name_start: 1232,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b5,
        name_start: 1239,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b6,
        name_start: 1245,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b7,
        name_start: 1251,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b9,
        name_start: 1256,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ba,
        name_start: 1262,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001bb,
        name_start: 1270,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001bc,
        name_start: 1276,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001bd,
        name_start: 1282,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001be,
        name_start: 1293,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001bf,
        name_start: 1299,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c0,
        name_start: 1308,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c3,
        name_start: 1314,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c5,
        name_start: 1320,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c6,
        name_start: 1326,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c8,
        name_start: 1332,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ca,
        name_start: 1338,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001cc,
        name_start: 1345,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001cf,
        name_start: 1351,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d0,
        name_start: 1357,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d1,
        name_start: 1364,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d2,
        name_start: 1370,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d5,
        name_start: 1376,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d8,
        name_start: 1388,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d9,
        name_start: 1394,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001db,
        name_start: 1399,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001de,
        name_start: 1411,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001e0,
        name_start: 1419,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001e3,
        name_start: 1425,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001e5,
        name_start: 1431,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001e6,
        name_start: 1437,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001e8,
        name_start: 1443,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ea,
        name_start: 1449,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ec,
        name_start: 1456,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ef,
        name_start: 1462,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f0,
        name_start: 1468,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f1,
        name_start: 1475,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f2,
        name_start: 1481,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f5,
        name_start: 1487,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f8,
        name_start: 1499,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f9,
        name_start: 1505,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001fb,
        name_start: 1510,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001fe,
        name_start: 1522,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ff,
        name_start: 1530,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002a1,
        name_start: 1538,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002a6,
        name_start: 1545,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002a9,
        name_start: 1556,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002ab,
        name_start: 1565,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002ac,
        name_start: 1571,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002b1,
        name_start: 1582,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002b6,
        name_start: 1589,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002b9,
        name_start: 1600,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002bb,
        name_start: 1608,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002bc,
        name_start: 1614,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002c5,
        name_start: 1625,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002c6,
        name_start: 1634,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002d5,
        name_start: 1645,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002d8,
        name_start: 1654,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002dd,
        name_start: 1665,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002de,
        name_start: 1671,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002e5,
        name_start: 1682,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002e6,
        name_start: 1691,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002f5,
        name_start: 1702,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002f8,
        name_start: 1711,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002fd,
        name_start: 1722,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002fe,
        name_start: 1728,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a2,
        name_start: 1739,
        name_len: 3,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000118,
        name_start: 1742,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a3,
        name_start: 1747,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a5,
        name_start: 1755,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a6,
        name_start: 1761,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003aa,
        name_start: 1769,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003ab,
        name_start: 1776,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003ac,
        name_start: 1784,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003b3,
        name_start: 1790,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003b5,
        name_start: 1798,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003b6,
        name_start: 1804,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003ba,
        name_start: 1812,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003bb,
        name_start: 1819,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003bc,
        name_start: 1827,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003bd,
        name_start: 1833,
        name_len: 3,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003bf,
        name_start: 1836,
        name_len: 3,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003c0,
        name_start: 1839,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003c7,
        name_start: 1846,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003cc,
        name_start: 1853,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003cf,
        name_start: 1862,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003d1,
        name_start: 1869,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003d2,
        name_start: 1877,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003d3,
        name_start: 1884,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003d9,
        name_start: 1892,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003dd,
        name_start: 1899,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003de,
        name_start: 1905,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003e0,
        name_start: 1912,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003e7,
        name_start: 1919,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003ec,
        name_start: 1926,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003ef,
        name_start: 1935,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003f1,
        name_start: 1942,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003f2,
        name_start: 1950,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003f3,
        name_start: 1957,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003f9,
        name_start: 1965,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003fd,
        name_start: 1972,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003fe,
        name_start: 1978,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000047e,
        name_start: 1985,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a1,
        name_start: 1993,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a2,
        name_start: 2006,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a3,
        name_start: 2025,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a4,
        name_start: 2044,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a5,
        name_start: 2054,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000141,
        name_start: 2070,
        name_len: 14,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a6,
        name_start: 2084,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a7,
        name_start: 2091,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a8,
        name_start: 2097,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a9,
        name_start: 2103,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004aa,
        name_start: 2109,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ab,
        name_start: 2115,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ac,
        name_start: 2121,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ad,
        name_start: 2128,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ae,
        name_start: 2135,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004af,
        name_start: 2142,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000014c,
        name_start: 2150,
        name_len: 7,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b0,
        name_start: 2157,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b1,
        name_start: 2171,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b2,
        name_start: 2177,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b3,
        name_start: 2183,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b4,
        name_start: 2189,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b5,
        name_start: 2195,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b6,
        name_start: 2201,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b7,
        name_start: 2208,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b8,
        name_start: 2215,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b9,
        name_start: 2222,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ba,
        name_start: 2229,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004bb,
        name_start: 2236,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004bc,
        name_start: 2243,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004bd,
        name_start: 2251,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004be,
        name_start: 2258,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004bf,
        name_start: 2265,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c0,
        name_start: 2272,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c1,
        name_start: 2279,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000015f,
        name_start: 2287,
        name_len: 7,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c2,
        name_start: 2294,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000161,
        name_start: 2302,
        name_len: 7,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c3,
        name_start: 2309,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c4,
        name_start: 2316,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c5,
        name_start: 2323,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c6,
        name_start: 2330,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c7,
        name_start: 2337,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c8,
        name_start: 2344,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c9,
        name_start: 2351,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ca,
        name_start: 2358,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cb,
        name_start: 2365,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cc,
        name_start: 2372,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000016c,
        name_start: 2379,
        name_len: 7,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cd,
        name_start: 2386,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ce,
        name_start: 2393,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cf,
        name_start: 2400,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d0,
        name_start: 2407,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d1,
        name_start: 2414,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d2,
        name_start: 2421,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d3,
        name_start: 2428,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d4,
        name_start: 2435,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d5,
        name_start: 2442,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d6,
        name_start: 2449,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d7,
        name_start: 2456,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d8,
        name_start: 2463,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d9,
        name_start: 2470,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004da,
        name_start: 2477,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004db,
        name_start: 2484,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004dc,
        name_start: 2491,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004dd,
        name_start: 2498,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004de,
        name_start: 2504,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004df,
        name_start: 2515,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ac,
        name_start: 2530,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005bb,
        name_start: 2542,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005bf,
        name_start: 2558,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c1,
        name_start: 2578,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c2,
        name_start: 2590,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c3,
        name_start: 2608,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c4,
        name_start: 2626,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c5,
        name_start: 2643,
        name_len: 21,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c6,
        name_start: 2664,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c7,
        name_start: 2681,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c8,
        name_start: 2692,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c9,
        name_start: 2702,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ca,
        name_start: 2719,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005cb,
        name_start: 2729,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005cc,
        name_start: 2740,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005cd,
        name_start: 2751,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ce,
        name_start: 2761,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005cf,
        name_start: 2772,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d0,
        name_start: 2782,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d1,
        name_start: 2793,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d2,
        name_start: 2802,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d3,
        name_start: 2813,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d4,
        name_start: 2824,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d5,
        name_start: 2836,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d6,
        name_start: 2846,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d7,
        name_start: 2856,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d8,
        name_start: 2866,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d9,
        name_start: 2876,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005da,
        name_start: 2886,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e0,
        name_start: 2898,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e1,
        name_start: 2912,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e2,
        name_start: 2922,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e3,
        name_start: 2932,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e4,
        name_start: 2942,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e5,
        name_start: 2952,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e6,
        name_start: 2963,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e7,
        name_start: 2974,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a5,
        name_start: 2983,
        name_len: 10,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e8,
        name_start: 2993,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e9,
        name_start: 3003,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ea,
        name_start: 3021,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005eb,
        name_start: 3031,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ec,
        name_start: 3046,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ed,
        name_start: 3061,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ee,
        name_start: 3076,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ef,
        name_start: 3088,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005f0,
        name_start: 3100,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005f1,
        name_start: 3112,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005f2,
        name_start: 3125,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a1,
        name_start: 3137,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a2,
        name_start: 3148,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a3,
        name_start: 3161,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a4,
        name_start: 3172,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b5,
        name_start: 3184,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a5,
        name_start: 3195,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a6,
        name_start: 3208,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b8,
        name_start: 3219,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a7,
        name_start: 3229,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ba,
        name_start: 3241,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a8,
        name_start: 3252,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001bc,
        name_start: 3263,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a9,
        name_start: 3273,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001be,
        name_start: 3285,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006aa,
        name_start: 3296,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c0,
        name_start: 3308,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ab,
        name_start: 3319,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ac,
        name_start: 3331,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ad,
        name_start: 3344,
        name_len: 25,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ae,
        name_start: 3369,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006af,
        name_start: 3388,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c6,
        name_start: 3401,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b0,
        name_start: 3412,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b1,
        name_start: 3422,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b2,
        name_start: 3433,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b3,
        name_start: 3446,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b4,
        name_start: 3457,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001cc,
        name_start: 3469,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b5,
        name_start: 3480,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b6,
        name_start: 3493,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001cf,
        name_start: 3504,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b7,
        name_start: 3514,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d1,
        name_start: 3526,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b8,
        name_start: 3537,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d3,
        name_start: 3548,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b9,
        name_start: 3558,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d5,
        name_start: 3570,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ba,
        name_start: 3581,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d7,
        name_start: 3593,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006bb,
        name_start: 3604,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006bc,
        name_start: 3616,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006bd,
        name_start: 3629,
        name_len: 25,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006be,
        name_start: 3654,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006bf,
        name_start: 3673,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001dd,
        name_start: 3686,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c0,
        name_start: 3697,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c1,
        name_start: 3708,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c2,
        name_start: 3718,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c3,
        name_start: 3729,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c4,
        name_start: 3741,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c5,
        name_start: 3752,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c6,
        name_start: 3763,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c7,
        name_start: 3774,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c8,
        name_start: 3786,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c9,
        name_start: 3797,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ca,
        name_start: 3807,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006cb,
        name_start: 3822,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006cc,
        name_start: 3833,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006cd,
        name_start: 3844,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ce,
        name_start: 3855,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006cf,
        name_start: 3866,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d0,
        name_start: 3876,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d1,
        name_start: 3887,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d2,
        name_start: 3898,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d3,
        name_start: 3909,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d4,
        name_start: 3920,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d5,
        name_start: 3931,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d6,
        name_start: 3941,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d7,
        name_start: 3953,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d8,
        name_start: 3964,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d9,
        name_start: 3981,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006da,
        name_start: 3994,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006db,
        name_start: 4005,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006dc,
        name_start: 4017,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006dd,
        name_start: 4027,
        name_len: 14,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006de,
        name_start: 4041,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006df,
        name_start: 4053,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e0,
        name_start: 4070,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e1,
        name_start: 4081,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e2,
        name_start: 4091,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e3,
        name_start: 4102,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e4,
        name_start: 4114,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e5,
        name_start: 4125,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e6,
        name_start: 4136,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e7,
        name_start: 4147,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e8,
        name_start: 4159,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e9,
        name_start: 4170,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ea,
        name_start: 4180,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006eb,
        name_start: 4195,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ec,
        name_start: 4206,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ed,
        name_start: 4217,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ee,
        name_start: 4228,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ef,
        name_start: 4239,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f0,
        name_start: 4249,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f1,
        name_start: 4260,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f2,
        name_start: 4271,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f3,
        name_start: 4282,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f4,
        name_start: 4293,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f5,
        name_start: 4304,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f6,
        name_start: 4314,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f7,
        name_start: 4326,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f8,
        name_start: 4337,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f9,
        name_start: 4354,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006fa,
        name_start: 4367,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006fb,
        name_start: 4378,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006fc,
        name_start: 4390,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006fd,
        name_start: 4400,
        name_len: 14,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006fe,
        name_start: 4414,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ff,
        name_start: 4426,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a1,
        name_start: 4443,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a2,
        name_start: 4460,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a3,
        name_start: 4479,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a4,
        name_start: 4494,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a5,
        name_start: 4510,
        name_len: 18,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000223,
        name_start: 4528,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a7,
        name_start: 4547,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a8,
        name_start: 4566,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a9,
        name_start: 4585,
        name_len: 21,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ab,
        name_start: 4606,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ae,
        name_start: 4623,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007af,
        name_start: 4643,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b1,
        name_start: 4657,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b2,
        name_start: 4674,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b3,
        name_start: 4693,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b4,
        name_start: 4708,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b5,
        name_start: 4724,
        name_len: 18,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b6,
        name_start: 4742,
        name_len: 24,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b7,
        name_start: 4766,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b8,
        name_start: 4785,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b9,
        name_start: 4804,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ba,
        name_start: 4825,
        name_len: 27,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007bb,
        name_start: 4852,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c1,
        name_start: 4869,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c2,
        name_start: 4880,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c3,
        name_start: 4890,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c4,
        name_start: 4901,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c5,
        name_start: 4912,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c6,
        name_start: 4925,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c7,
        name_start: 4935,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c8,
        name_start: 4944,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c9,
        name_start: 4955,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ca,
        name_start: 4965,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007cb,
        name_start: 4976,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000240,
        name_start: 4987,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007cc,
        name_start: 4999,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007cd,
        name_start: 5007,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ce,
        name_start: 5015,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007cf,
        name_start: 5023,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d0,
        name_start: 5036,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d1,
        name_start: 5044,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d2,
        name_start: 5053,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d4,
        name_start: 5064,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d5,
        name_start: 5073,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d6,
        name_start: 5086,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d7,
        name_start: 5095,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d8,
        name_start: 5104,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d9,
        name_start: 5113,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e1,
        name_start: 5124,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e2,
        name_start: 5135,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e3,
        name_start: 5145,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e4,
        name_start: 5156,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e5,
        name_start: 5167,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e6,
        name_start: 5180,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e7,
        name_start: 5190,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e8,
        name_start: 5199,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e9,
        name_start: 5210,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ea,
        name_start: 5220,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007eb,
        name_start: 5231,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000259,
        name_start: 5242,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ec,
        name_start: 5254,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ed,
        name_start: 5262,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ee,
        name_start: 5270,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ef,
        name_start: 5278,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f0,
        name_start: 5291,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f1,
        name_start: 5299,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f2,
        name_start: 5308,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f3,
        name_start: 5319,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f4,
        name_start: 5340,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f5,
        name_start: 5349,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f6,
        name_start: 5362,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f7,
        name_start: 5371,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f8,
        name_start: 5380,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f9,
        name_start: 5389,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a1,
        name_start: 5400,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a2,
        name_start: 5411,
        name_len: 14,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a3,
        name_start: 5425,
        name_len: 14,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a4,
        name_start: 5439,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a5,
        name_start: 5450,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a6,
        name_start: 5461,
        name_len: 13,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a7,
        name_start: 5474,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a8,
        name_start: 5490,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a9,
        name_start: 5506,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008aa,
        name_start: 5523,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ab,
        name_start: 5540,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ac,
        name_start: 5553,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ad,
        name_start: 5566,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ae,
        name_start: 5580,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008af,
        name_start: 5594,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b0,
        name_start: 5614,
        name_len: 21,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b1,
        name_start: 5635,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b2,
        name_start: 5651,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b3,
        name_start: 5667,
        name_len: 25,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b4,
        name_start: 5692,
        name_len: 25,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b5,
        name_start: 5717,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b6,
        name_start: 5734,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b7,
        name_start: 5751,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008bc,
        name_start: 5771,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008bd,
        name_start: 5784,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008be,
        name_start: 5792,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008bf,
        name_start: 5808,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c0,
        name_start: 5816,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c1,
        name_start: 5825,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c2,
        name_start: 5834,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c5,
        name_start: 5842,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c8,
        name_start: 5847,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c9,
        name_start: 5858,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008cd,
        name_start: 5870,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ce,
        name_start: 5878,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008cf,
        name_start: 5885,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008d6,
        name_start: 5894,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008da,
        name_start: 5901,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008db,
        name_start: 5911,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008dc,
        name_start: 5919,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008dd,
        name_start: 5931,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008de,
        name_start: 5936,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008df,
        name_start: 5946,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ef,
        name_start: 5955,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008f6,
        name_start: 5972,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008fb,
        name_start: 5980,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008fc,
        name_start: 5989,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008fd,
        name_start: 5996,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008fe,
        name_start: 6006,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009df,
        name_start: 6015,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e0,
        name_start: 6020,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e1,
        name_start: 6032,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e2,
        name_start: 6044,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e3,
        name_start: 6046,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e4,
        name_start: 6048,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e5,
        name_start: 6050,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e8,
        name_start: 6052,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e9,
        name_start: 6054,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ea,
        name_start: 6056,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009eb,
        name_start: 6070,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ec,
        name_start: 6083,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ed,
        name_start: 6095,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ee,
        name_start: 6108,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ef,
        name_start: 6121,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f0,
        name_start: 6135,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f1,
        name_start: 6149,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f2,
        name_start: 6163,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f3,
        name_start: 6177,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f4,
        name_start: 6191,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f5,
        name_start: 6196,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f6,
        name_start: 6202,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f7,
        name_start: 6206,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f8,
        name_start: 6210,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa1,
        name_start: 6217,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa2,
        name_start: 6224,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa3,
        name_start: 6231,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa4,
        name_start: 6239,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa5,
        name_start: 6247,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa6,
        name_start: 6257,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa7,
        name_start: 6267,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa8,
        name_start: 6276,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa9,
        name_start: 6285,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aaa,
        name_start: 6291,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aac,
        name_start: 6297,
        name_len: 11,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aae,
        name_start: 6308,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aaf,
        name_start: 6316,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab0,
        name_start: 6331,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab1,
        name_start: 6339,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab2,
        name_start: 6348,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab3,
        name_start: 6356,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab4,
        name_start: 6365,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab5,
        name_start: 6376,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab6,
        name_start: 6386,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab7,
        name_start: 6394,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab8,
        name_start: 6404,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000abb,
        name_start: 6410,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000abc,
        name_start: 6417,
        name_len: 16,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000abd,
        name_start: 6433,
        name_len: 12,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000abe,
        name_start: 6445,
        name_len: 17,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000abf,
        name_start: 6462,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ac3,
        name_start: 6468,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ac4,
        name_start: 6477,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ac5,
        name_start: 6489,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ac6,
        name_start: 6500,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ac9,
        name_start: 6512,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aca,
        name_start: 6521,
        name_len: 13,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000acb,
        name_start: 6534,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000acc,
        name_start: 6551,
        name_len: 16,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000acd,
        name_start: 6567,
        name_len: 17,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ace,
        name_start: 6584,
        name_len: 12,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000acf,
        name_start: 6596,
        name_len: 15,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad0,
        name_start: 6611,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad1,
        name_start: 6630,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad2,
        name_start: 6650,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad3,
        name_start: 6669,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad4,
        name_start: 6689,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad5,
        name_start: 6701,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad6,
        name_start: 6709,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad7,
        name_start: 6716,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad9,
        name_start: 6723,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ada,
        name_start: 6733,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000adb,
        name_start: 6741,
        name_len: 16,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000adc,
        name_start: 6757,
        name_len: 19,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000add,
        name_start: 6776,
        name_len: 20,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ade,
        name_start: 6796,
        name_len: 14,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000adf,
        name_start: 6810,
        name_len: 12,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae0,
        name_start: 6822,
        name_len: 16,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae1,
        name_start: 6838,
        name_len: 18,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae2,
        name_start: 6856,
        name_len: 14,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae3,
        name_start: 6870,
        name_len: 15,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae4,
        name_start: 6885,
        name_len: 17,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae5,
        name_start: 6902,
        name_len: 8,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae6,
        name_start: 6910,
        name_len: 18,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae7,
        name_start: 6928,
        name_len: 16,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae8,
        name_start: 6944,
        name_len: 17,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae9,
        name_start: 6961,
        name_len: 19,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aea,
        name_start: 6980,
        name_len: 11,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aeb,
        name_start: 6991,
        name_len: 12,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aec,
        name_start: 7003,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aed,
        name_start: 7007,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aee,
        name_start: 7014,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af0,
        name_start: 7019,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af1,
        name_start: 7031,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af2,
        name_start: 7037,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af3,
        name_start: 7049,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af4,
        name_start: 7058,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af5,
        name_start: 7069,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af6,
        name_start: 7081,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af7,
        name_start: 7092,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af8,
        name_start: 7102,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af9,
        name_start: 7114,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000afa,
        name_start: 7123,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000afb,
        name_start: 7140,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000afc,
        name_start: 7159,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000afd,
        name_start: 7164,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000afe,
        name_start: 7182,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aff,
        name_start: 7200,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ba3,
        name_start: 7206,
        name_len: 9,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ba6,
        name_start: 7215,
        name_len: 10,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ba8,
        name_start: 7225,
        name_len: 9,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ba9,
        name_start: 7234,
        name_len: 7,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bc0,
        name_start: 7241,
        name_len: 7,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bc2,
        name_start: 7248,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bc3,
        name_start: 7256,
        name_len: 6,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bc4,
        name_start: 7262,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bc6,
        name_start: 7271,
        name_len: 8,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bca,
        name_start: 7279,
        name_len: 3,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bcc,
        name_start: 7282,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bce,
        name_start: 7286,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bcf,
        name_start: 7292,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bd3,
        name_start: 7298,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bd6,
        name_start: 7305,
        name_len: 8,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bd8,
        name_start: 7313,
        name_len: 9,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bda,
        name_start: 7322,
        name_len: 8,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bdc,
        name_start: 7330,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bfc,
        name_start: 7338,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cdf,
        name_start: 7347,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce0,
        name_start: 7367,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce1,
        name_start: 7379,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000031b,
        name_start: 7389,
        name_len: 11,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce2,
        name_start: 7400,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000031d,
        name_start: 7412,
        name_len: 13,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce3,
        name_start: 7425,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000031f,
        name_start: 7437,
        name_len: 13,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce4,
        name_start: 7450,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce5,
        name_start: 7459,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce6,
        name_start: 7469,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000323,
        name_start: 7480,
        name_len: 12,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce7,
        name_start: 7492,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000325,
        name_start: 7503,
        name_len: 10,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce8,
        name_start: 7513,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000327,
        name_start: 7523,
        name_len: 11,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce9,
        name_start: 7534,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cea,
        name_start: 7544,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ceb,
        name_start: 7560,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cec,
        name_start: 7571,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ced,
        name_start: 7583,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cee,
        name_start: 7598,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cef,
        name_start: 7608,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf0,
        name_start: 7623,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf1,
        name_start: 7633,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000331,
        name_start: 7646,
        name_len: 13,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf2,
        name_start: 7659,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf3,
        name_start: 7670,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf4,
        name_start: 7684,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf5,
        name_start: 7693,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000336,
        name_start: 7709,
        name_len: 16,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf6,
        name_start: 7725,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000338,
        name_start: 7736,
        name_len: 11,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf7,
        name_start: 7747,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000033a,
        name_start: 7758,
        name_len: 10,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf8,
        name_start: 7768,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf9,
        name_start: 7779,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cfa,
        name_start: 7790,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000033e,
        name_start: 7800,
        name_len: 10,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da1,
        name_start: 7810,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da2,
        name_start: 7820,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da3,
        name_start: 7832,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da4,
        name_start: 7845,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da5,
        name_start: 7858,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da6,
        name_start: 7870,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da7,
        name_start: 7885,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da8,
        name_start: 7896,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da9,
        name_start: 7908,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000daa,
        name_start: 7921,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dab,
        name_start: 7934,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dac,
        name_start: 7943,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dad,
        name_start: 7955,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dae,
        name_start: 7966,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000daf,
        name_start: 7978,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db0,
        name_start: 7990,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db1,
        name_start: 8002,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db2,
        name_start: 8020,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db3,
        name_start: 8035,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db4,
        name_start: 8045,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db5,
        name_start: 8055,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db6,
        name_start: 8065,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db7,
        name_start: 8078,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db8,
        name_start: 8092,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db9,
        name_start: 8105,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dba,
        name_start: 8114,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dbb,
        name_start: 8127,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dbc,
        name_start: 8137,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dbd,
        name_start: 8150,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dbe,
        name_start: 8159,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dbf,
        name_start: 8171,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc0,
        name_start: 8181,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc1,
        name_start: 8196,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc2,
        name_start: 8205,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc3,
        name_start: 8215,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc4,
        name_start: 8225,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc5,
        name_start: 8232,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc6,
        name_start: 8243,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc7,
        name_start: 8250,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc8,
        name_start: 8261,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc9,
        name_start: 8272,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dca,
        name_start: 8283,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dcb,
        name_start: 8293,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dcc,
        name_start: 8303,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dcd,
        name_start: 8315,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dce,
        name_start: 8324,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dcf,
        name_start: 8337,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd0,
        name_start: 8351,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd1,
        name_start: 8361,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd2,
        name_start: 8376,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd3,
        name_start: 8387,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd4,
        name_start: 8398,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd5,
        name_start: 8408,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd6,
        name_start: 8419,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd7,
        name_start: 8430,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd8,
        name_start: 8442,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd9,
        name_start: 8452,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dda,
        name_start: 8463,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dde,
        name_start: 8475,
        name_len: 22,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ddf,
        name_start: 8497,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de0,
        name_start: 8506,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de1,
        name_start: 8516,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de2,
        name_start: 8527,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de3,
        name_start: 8537,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de4,
        name_start: 8555,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de5,
        name_start: 8574,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de6,
        name_start: 8590,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de7,
        name_start: 8603,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de8,
        name_start: 8617,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de9,
        name_start: 8627,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dea,
        name_start: 8638,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000deb,
        name_start: 8649,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dec,
        name_start: 8665,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ded,
        name_start: 8681,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df0,
        name_start: 8694,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df1,
        name_start: 8705,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df2,
        name_start: 8717,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df3,
        name_start: 8729,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df4,
        name_start: 8740,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df5,
        name_start: 8750,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df6,
        name_start: 8760,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df7,
        name_start: 8771,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df8,
        name_start: 8783,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df9,
        name_start: 8795,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea1,
        name_start: 8806,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea2,
        name_start: 8819,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea3,
        name_start: 8837,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea4,
        name_start: 8854,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea5,
        name_start: 8866,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea6,
        name_start: 8883,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea7,
        name_start: 8900,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea8,
        name_start: 8913,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea9,
        name_start: 8931,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eaa,
        name_start: 8943,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eab,
        name_start: 8961,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eac,
        name_start: 8978,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ead,
        name_start: 8995,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eae,
        name_start: 9011,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eaf,
        name_start: 9028,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb0,
        name_start: 9046,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb1,
        name_start: 9063,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb2,
        name_start: 9075,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb3,
        name_start: 9087,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb4,
        name_start: 9104,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb5,
        name_start: 9120,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb6,
        name_start: 9131,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb7,
        name_start: 9147,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb8,
        name_start: 9159,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb9,
        name_start: 9171,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eba,
        name_start: 9188,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ebb,
        name_start: 9200,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ebc,
        name_start: 9213,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ebd,
        name_start: 9225,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ebe,
        name_start: 9238,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ebf,
        name_start: 9250,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec0,
        name_start: 9258,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec1,
        name_start: 9267,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec2,
        name_start: 9276,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec3,
        name_start: 9286,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec4,
        name_start: 9295,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec5,
        name_start: 9303,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec6,
        name_start: 9313,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec7,
        name_start: 9322,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec8,
        name_start: 9330,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec9,
        name_start: 9339,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eca,
        name_start: 9349,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ecb,
        name_start: 9358,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ecc,
        name_start: 9367,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ecd,
        name_start: 9375,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ece,
        name_start: 9385,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ecf,
        name_start: 9394,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed0,
        name_start: 9403,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed1,
        name_start: 9412,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed2,
        name_start: 9421,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed3,
        name_start: 9430,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed4,
        name_start: 9438,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed5,
        name_start: 9453,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed6,
        name_start: 9473,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed7,
        name_start: 9492,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed8,
        name_start: 9506,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed9,
        name_start: 9525,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eda,
        name_start: 9544,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000edb,
        name_start: 9559,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000edc,
        name_start: 9573,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000edd,
        name_start: 9593,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ede,
        name_start: 9612,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000edf,
        name_start: 9631,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee0,
        name_start: 9649,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee1,
        name_start: 9668,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee2,
        name_start: 9688,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee3,
        name_start: 9707,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee4,
        name_start: 9721,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee5,
        name_start: 9735,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee6,
        name_start: 9753,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee7,
        name_start: 9766,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee8,
        name_start: 9784,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee9,
        name_start: 9798,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eea,
        name_start: 9812,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eeb,
        name_start: 9826,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eec,
        name_start: 9841,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eed,
        name_start: 9855,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eee,
        name_start: 9870,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eef,
        name_start: 9884,
        name_len: 23,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef0,
        name_start: 9907,
        name_len: 24,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef1,
        name_start: 9931,
        name_len: 24,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef2,
        name_start: 9955,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef3,
        name_start: 9969,
        name_len: 24,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef4,
        name_start: 9993,
        name_len: 25,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef5,
        name_start: 10018,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef6,
        name_start: 10036,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef7,
        name_start: 10048,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef8,
        name_start: 10061,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef9,
        name_start: 10077,
        name_len: 26,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000efa,
        name_start: 10103,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eff,
        name_start: 10123,
        name_len: 10,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000013bc,
        name_start: 10133,
        name_len: 2,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000013bd,
        name_start: 10135,
        name_len: 2,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000013be,
        name_start: 10137,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000020ac,
        name_start: 10147,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd01,
        name_start: 10155,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd02,
        name_start: 10169,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd03,
        name_start: 10183,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd04,
        name_start: 10194,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd05,
        name_start: 10204,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd06,
        name_start: 10216,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd07,
        name_start: 10229,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd08,
        name_start: 10244,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd09,
        name_start: 10254,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0a,
        name_start: 10263,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0b,
        name_start: 10271,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0c,
        name_start: 10279,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0d,
        name_start: 10287,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0e,
        name_start: 10296,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0f,
        name_start: 10305,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd10,
        name_start: 10321,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd11,
        name_start: 10335,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd12,
        name_start: 10348,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd13,
        name_start: 10357,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd14,
        name_start: 10367,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd15,
        name_start: 10376,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd16,
        name_start: 10385,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd17,
        name_start: 10394,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd18,
        name_start: 10404,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd19,
        name_start: 10415,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd1a,
        name_start: 10432,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd1b,
        name_start: 10447,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd1c,
        name_start: 10460,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd1d,
        name_start: 10477,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd1e,
        name_start: 10493,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe01,
        name_start: 10503,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe02,
        name_start: 10511,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe03,
        name_start: 10527,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe04,
        name_start: 10543,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe05,
        name_start: 10559,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe06,
        name_start: 10574,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe07,
        name_start: 10589,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe08,
        name_start: 10603,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe09,
        name_start: 10617,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0a,
        name_start: 10636,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0b,
        name_start: 10650,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0c,
        name_start: 10669,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0d,
        name_start: 10684,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0e,
        name_start: 10704,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0f,
        name_start: 10718,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe11,
        name_start: 10737,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe12,
        name_start: 10753,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe13,
        name_start: 10769,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe20,
        name_start: 10784,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe21,
        name_start: 10796,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe22,
        name_start: 10812,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe23,
        name_start: 10830,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe24,
        name_start: 10849,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe25,
        name_start: 10870,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe26,
        name_start: 10892,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe27,
        name_start: 10915,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe28,
        name_start: 10934,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe29,
        name_start: 10954,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2a,
        name_start: 10977,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2b,
        name_start: 11001,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2c,
        name_start: 11025,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2d,
        name_start: 11045,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2e,
        name_start: 11066,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2f,
        name_start: 11084,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe30,
        name_start: 11104,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe31,
        name_start: 11128,
        name_len: 27,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe32,
        name_start: 11155,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe33,
        name_start: 11168,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe34,
        name_start: 11185,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe50,
        name_start: 11194,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe51,
        name_start: 11204,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe52,
        name_start: 11214,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe53,
        name_start: 11229,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000043b,
        name_start: 11239,
        name_len: 16,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe54,
        name_start: 11255,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe55,
        name_start: 11266,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe56,
        name_start: 11276,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe57,
        name_start: 11289,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe58,
        name_start: 11303,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe59,
        name_start: 11317,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5a,
        name_start: 11333,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5b,
        name_start: 11343,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5c,
        name_start: 11355,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5d,
        name_start: 11366,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5e,
        name_start: 11375,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5f,
        name_start: 11392,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe60,
        name_start: 11413,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe61,
        name_start: 11426,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe62,
        name_start: 11435,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe63,
        name_start: 11444,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe64,
        name_start: 11455,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000044d,
        name_start: 11470,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe65,
        name_start: 11480,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000044f,
        name_start: 11503,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe66,
        name_start: 11513,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe67,
        name_start: 11529,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe68,
        name_start: 11543,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe69,
        name_start: 11559,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6a,
        name_start: 11579,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6b,
        name_start: 11594,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6c,
        name_start: 11609,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6d,
        name_start: 11628,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6e,
        name_start: 11646,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6f,
        name_start: 11661,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe70,
        name_start: 11674,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe71,
        name_start: 11688,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe72,
        name_start: 11711,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe73,
        name_start: 11728,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe74,
        name_start: 11743,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe75,
        name_start: 11760,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe76,
        name_start: 11777,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe77,
        name_start: 11793,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe78,
        name_start: 11815,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe79,
        name_start: 11830,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe7a,
        name_start: 11845,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe80,
        name_start: 11863,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe81,
        name_start: 11869,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe82,
        name_start: 11875,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe83,
        name_start: 11881,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe84,
        name_start: 11887,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe85,
        name_start: 11893,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe86,
        name_start: 11899,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe87,
        name_start: 11905,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe88,
        name_start: 11911,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe89,
        name_start: 11917,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe8a,
        name_start: 11923,
        name_len: 16,
        flags: 0 | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000470,
        name_start: 11939,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe8b,
        name_start: 11949,
        name_len: 18,
        flags: 0 | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000472,
        name_start: 11967,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe8c,
        name_start: 11977,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe8d,
        name_start: 11987,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe90,
        name_start: 11997,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe91,
        name_start: 12009,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe92,
        name_start: 12031,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe93,
        name_start: 12053,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea0,
        name_start: 12076,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea1,
        name_start: 12078,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea2,
        name_start: 12080,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea3,
        name_start: 12082,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea4,
        name_start: 12085,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea5,
        name_start: 12088,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed0,
        name_start: 12091,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed1,
        name_start: 12111,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed2,
        name_start: 12130,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed4,
        name_start: 12149,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed5,
        name_start: 12168,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee0,
        name_start: 12184,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee1,
        name_start: 12196,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee2,
        name_start: 12209,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee3,
        name_start: 12219,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee4,
        name_start: 12231,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee5,
        name_start: 12245,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee6,
        name_start: 12260,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee7,
        name_start: 12276,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee8,
        name_start: 12293,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee9,
        name_start: 12312,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feea,
        name_start: 12327,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feeb,
        name_start: 12342,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feec,
        name_start: 12357,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feed,
        name_start: 12372,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feee,
        name_start: 12387,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feef,
        name_start: 12408,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef0,
        name_start: 12425,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef1,
        name_start: 12442,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef2,
        name_start: 12459,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef3,
        name_start: 12476,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef4,
        name_start: 12493,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef5,
        name_start: 12510,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef6,
        name_start: 12523,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef7,
        name_start: 12536,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef8,
        name_start: 12549,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef9,
        name_start: 12562,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefa,
        name_start: 12580,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefb,
        name_start: 12598,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefc,
        name_start: 12617,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefd,
        name_start: 12636,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff08,
        name_start: 12649,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff09,
        name_start: 12658,
        name_len: 3,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff0a,
        name_start: 12661,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff0b,
        name_start: 12669,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff0d,
        name_start: 12674,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff13,
        name_start: 12680,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff14,
        name_start: 12685,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff15,
        name_start: 12696,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff1b,
        name_start: 12703,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff20,
        name_start: 12709,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ac,
        name_start: 12718,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff21,
        name_start: 12728,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff22,
        name_start: 12733,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff23,
        name_start: 12741,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b0,
        name_start: 12752,
        name_len: 6,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff24,
        name_start: 12758,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff25,
        name_start: 12764,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff26,
        name_start: 12772,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff27,
        name_start: 12780,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff28,
        name_start: 12797,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff29,
        name_start: 12804,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2a,
        name_start: 12811,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2b,
        name_start: 12826,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2c,
        name_start: 12833,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2d,
        name_start: 12839,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2e,
        name_start: 12848,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2f,
        name_start: 12858,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff30,
        name_start: 12868,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff31,
        name_start: 12879,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff32,
        name_start: 12885,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff33,
        name_start: 12897,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff34,
        name_start: 12907,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff35,
        name_start: 12919,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff36,
        name_start: 12930,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff37,
        name_start: 12943,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c5,
        name_start: 12952,
        name_len: 12,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c5,
        name_start: 12964,
        name_len: 16,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff38,
        name_start: 12980,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff39,
        name_start: 12993,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3a,
        name_start: 13005,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3b,
        name_start: 13020,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3c,
        name_start: 13036,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cc,
        name_start: 13051,
        name_len: 22,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3d,
        name_start: 13073,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ce,
        name_start: 13090,
        name_len: 8,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ce,
        name_start: 13098,
        name_len: 24,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3e,
        name_start: 13122,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d1,
        name_start: 13139,
        name_len: 8,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d1,
        name_start: 13147,
        name_len: 24,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3f,
        name_start: 13171,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff50,
        name_start: 13185,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff51,
        name_start: 13189,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff52,
        name_start: 13193,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff53,
        name_start: 13195,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff54,
        name_start: 13200,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff55,
        name_start: 13204,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004da,
        name_start: 13209,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004da,
        name_start: 13216,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff56,
        name_start: 13225,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004dd,
        name_start: 13229,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004dd,
        name_start: 13238,
        name_len: 11,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff57,
        name_start: 13249,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff58,
        name_start: 13252,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff60,
        name_start: 13257,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff61,
        name_start: 13263,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004e3,
        name_start: 13268,
        name_len: 15,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff62,
        name_start: 13283,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff63,
        name_start: 13290,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff65,
        name_start: 13296,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004e7,
        name_start: 13300,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff66,
        name_start: 13307,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004e9,
        name_start: 13311,
        name_len: 8,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff67,
        name_start: 13319,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff68,
        name_start: 13323,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ec,
        name_start: 13327,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff69,
        name_start: 13334,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ee,
        name_start: 13340,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff6a,
        name_start: 13347,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff6b,
        name_start: 13351,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff7e,
        name_start: 13356,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f2,
        name_start: 13367,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f2,
        name_start: 13380,
        name_len: 15,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f2,
        name_start: 13395,
        name_len: 11,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f2,
        name_start: 13406,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f2,
        name_start: 13419,
        name_len: 12,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f2,
        name_start: 13431,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f2,
        name_start: 13444,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f2,
        name_start: 13457,
        name_len: 11,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff7f,
        name_start: 13468,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff80,
        name_start: 13476,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff89,
        name_start: 13484,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff8d,
        name_start: 13490,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff91,
        name_start: 13498,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff92,
        name_start: 13503,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff93,
        name_start: 13508,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff94,
        name_start: 13513,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff95,
        name_start: 13518,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff96,
        name_start: 13525,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff97,
        name_start: 13532,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff98,
        name_start: 13537,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff99,
        name_start: 13545,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9a,
        name_start: 13552,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000508,
        name_start: 13560,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9b,
        name_start: 13570,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000050a,
        name_start: 13577,
        name_len: 12,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9c,
        name_start: 13589,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9d,
        name_start: 13595,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9e,
        name_start: 13603,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9f,
        name_start: 13612,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffaa,
        name_start: 13621,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffab,
        name_start: 13632,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffac,
        name_start: 13638,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffad,
        name_start: 13650,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffae,
        name_start: 13661,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffaf,
        name_start: 13671,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb0,
        name_start: 13680,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb1,
        name_start: 13684,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb2,
        name_start: 13688,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb3,
        name_start: 13692,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb4,
        name_start: 13696,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb5,
        name_start: 13700,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb6,
        name_start: 13704,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb7,
        name_start: 13708,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb8,
        name_start: 13712,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb9,
        name_start: 13716,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffbd,
        name_start: 13720,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffbe,
        name_start: 13728,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffbf,
        name_start: 13730,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc0,
        name_start: 13732,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc1,
        name_start: 13734,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc2,
        name_start: 13736,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc3,
        name_start: 13738,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc4,
        name_start: 13740,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc5,
        name_start: 13742,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc6,
        name_start: 13744,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc7,
        name_start: 13746,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc8,
        name_start: 13749,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000052b,
        name_start: 13752,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc9,
        name_start: 13754,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000052d,
        name_start: 13757,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffca,
        name_start: 13759,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000052f,
        name_start: 13762,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcb,
        name_start: 13764,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000531,
        name_start: 13767,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcc,
        name_start: 13769,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000533,
        name_start: 13772,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcd,
        name_start: 13774,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000535,
        name_start: 13777,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffce,
        name_start: 13779,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000537,
        name_start: 13782,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcf,
        name_start: 13784,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000539,
        name_start: 13787,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd0,
        name_start: 13789,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000053b,
        name_start: 13792,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd1,
        name_start: 13794,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000053d,
        name_start: 13797,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd2,
        name_start: 13800,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000053f,
        name_start: 13803,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd3,
        name_start: 13805,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000541,
        name_start: 13808,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd4,
        name_start: 13810,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000543,
        name_start: 13813,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd5,
        name_start: 13815,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000545,
        name_start: 13818,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd6,
        name_start: 13820,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000547,
        name_start: 13823,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd7,
        name_start: 13825,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000549,
        name_start: 13828,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd8,
        name_start: 13830,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000054b,
        name_start: 13833,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd9,
        name_start: 13835,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000054d,
        name_start: 13838,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffda,
        name_start: 13840,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000054f,
        name_start: 13843,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdb,
        name_start: 13845,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000551,
        name_start: 13848,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdc,
        name_start: 13851,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000553,
        name_start: 13854,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdd,
        name_start: 13857,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000555,
        name_start: 13860,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffde,
        name_start: 13863,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000557,
        name_start: 13866,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdf,
        name_start: 13869,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000559,
        name_start: 13872,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe0,
        name_start: 13875,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000055b,
        name_start: 13878,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe1,
        name_start: 13881,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe2,
        name_start: 13888,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe3,
        name_start: 13895,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe4,
        name_start: 13904,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe5,
        name_start: 13913,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe6,
        name_start: 13922,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe7,
        name_start: 13932,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe8,
        name_start: 13938,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe9,
        name_start: 13944,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffea,
        name_start: 13949,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffeb,
        name_start: 13954,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffec,
        name_start: 13961,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffed,
        name_start: 13968,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffee,
        name_start: 13975,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff1,
        name_start: 13982,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff2,
        name_start: 13995,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff3,
        name_start: 14008,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff4,
        name_start: 14021,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff5,
        name_start: 14034,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff6,
        name_start: 14047,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff7,
        name_start: 14060,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff8,
        name_start: 14073,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff9,
        name_start: 14086,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fffa,
        name_start: 14099,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffff,
        name_start: 14113,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00ffffff,
        name_start: 14119,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100012c,
        name_start: 14129,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100012d,
        name_start: 14135,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000174,
        name_start: 14141,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000175,
        name_start: 14152,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000176,
        name_start: 14163,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000177,
        name_start: 14174,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100018f,
        name_start: 14185,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100019f,
        name_start: 14190,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001a0,
        name_start: 14197,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001a1,
        name_start: 14202,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001af,
        name_start: 14207,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b0,
        name_start: 14212,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b5,
        name_start: 14217,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b6,
        name_start: 14224,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b7,
        name_start: 14231,
        name_len: 3,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001d1,
        name_start: 14234,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001d2,
        name_start: 14240,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001e6,
        name_start: 14246,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001e7,
        name_start: 14252,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000259,
        name_start: 14258,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000275,
        name_start: 14263,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000292,
        name_start: 14270,
        name_len: 3,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000300,
        name_start: 14273,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000301,
        name_start: 14288,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000303,
        name_start: 14303,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000309,
        name_start: 14318,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000323,
        name_start: 14332,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000492,
        name_start: 14350,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000493,
        name_start: 14366,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000496,
        name_start: 14382,
        name_len: 22,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000497,
        name_start: 14404,
        name_len: 22,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049a,
        name_start: 14426,
        name_len: 21,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049b,
        name_start: 14447,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049c,
        name_start: 14468,
        name_len: 22,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049d,
        name_start: 14490,
        name_len: 22,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004a2,
        name_start: 14512,
        name_len: 21,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004a3,
        name_start: 14533,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ae,
        name_start: 14554,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004af,
        name_start: 14573,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b0,
        name_start: 14592,
        name_len: 23,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b1,
        name_start: 14615,
        name_len: 23,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b2,
        name_start: 14638,
        name_len: 21,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b3,
        name_start: 14659,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b6,
        name_start: 14680,
        name_len: 22,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b7,
        name_start: 14702,
        name_len: 22,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b8,
        name_start: 14724,
        name_len: 23,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b9,
        name_start: 14747,
        name_len: 23,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ba,
        name_start: 14770,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004bb,
        name_start: 14783,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004d8,
        name_start: 14796,
        name_len: 14,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004d9,
        name_start: 14810,
        name_len: 14,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e2,
        name_start: 14824,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e3,
        name_start: 14841,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e8,
        name_start: 14858,
        name_len: 14,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e9,
        name_start: 14872,
        name_len: 14,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ee,
        name_start: 14886,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ef,
        name_start: 14903,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000531,
        name_start: 14920,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000532,
        name_start: 14932,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000533,
        name_start: 14944,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000534,
        name_start: 14956,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000535,
        name_start: 14967,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000536,
        name_start: 14980,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000537,
        name_start: 14991,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000538,
        name_start: 15001,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000539,
        name_start: 15012,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053a,
        name_start: 15023,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053b,
        name_start: 15035,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053c,
        name_start: 15047,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053d,
        name_start: 15060,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053e,
        name_start: 15072,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053f,
        name_start: 15084,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000540,
        name_start: 15096,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000541,
        name_start: 15107,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000542,
        name_start: 15119,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000543,
        name_start: 15132,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000544,
        name_start: 15145,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000545,
        name_start: 15157,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000546,
        name_start: 15168,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000547,
        name_start: 15179,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000548,
        name_start: 15191,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000549,
        name_start: 15202,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054a,
        name_start: 15214,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054b,
        name_start: 15225,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054c,
        name_start: 15236,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054d,
        name_start: 15247,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054e,
        name_start: 15258,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054f,
        name_start: 15270,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000550,
        name_start: 15283,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000551,
        name_start: 15294,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000552,
        name_start: 15306,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000553,
        name_start: 15319,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000554,
        name_start: 15332,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000555,
        name_start: 15343,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000556,
        name_start: 15353,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055a,
        name_start: 15364,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055b,
        name_start: 15383,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d7,
        name_start: 15398,
        name_len: 15,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055c,
        name_start: 15413,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d9,
        name_start: 15428,
        name_len: 15,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055d,
        name_start: 15443,
        name_len: 24,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005db,
        name_start: 15467,
        name_len: 12,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055e,
        name_start: 15479,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005dd,
        name_start: 15496,
        name_len: 15,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000561,
        name_start: 15511,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000562,
        name_start: 15523,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000563,
        name_start: 15535,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000564,
        name_start: 15547,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000565,
        name_start: 15558,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000566,
        name_start: 15571,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000567,
        name_start: 15582,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000568,
        name_start: 15592,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000569,
        name_start: 15603,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056a,
        name_start: 15614,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056b,
        name_start: 15626,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056c,
        name_start: 15638,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056d,
        name_start: 15651,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056e,
        name_start: 15663,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056f,
        name_start: 15675,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000570,
        name_start: 15687,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000571,
        name_start: 15698,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000572,
        name_start: 15710,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000573,
        name_start: 15723,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000574,
        name_start: 15736,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000575,
        name_start: 15748,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000576,
        name_start: 15759,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000577,
        name_start: 15770,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000578,
        name_start: 15782,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000579,
        name_start: 15793,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057a,
        name_start: 15805,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057b,
        name_start: 15816,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057c,
        name_start: 15827,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057d,
        name_start: 15838,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057e,
        name_start: 15849,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057f,
        name_start: 15861,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000580,
        name_start: 15874,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000581,
        name_start: 15885,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000582,
        name_start: 15897,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000583,
        name_start: 15910,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000584,
        name_start: 15923,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000585,
        name_start: 15934,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000586,
        name_start: 15944,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000587,
        name_start: 15955,
        name_len: 20,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000589,
        name_start: 15975,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000606,
        name_start: 15993,
        name_len: 17,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100058a,
        name_start: 16010,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000608,
        name_start: 16025,
        name_len: 17,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000653,
        name_start: 16042,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000654,
        name_start: 16060,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000655,
        name_start: 16078,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000660,
        name_start: 16096,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000661,
        name_start: 16104,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000662,
        name_start: 16112,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000663,
        name_start: 16120,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000664,
        name_start: 16128,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000665,
        name_start: 16136,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000666,
        name_start: 16144,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000667,
        name_start: 16152,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000668,
        name_start: 16160,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000669,
        name_start: 16168,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100066a,
        name_start: 16176,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000670,
        name_start: 16190,
        name_len: 23,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000679,
        name_start: 16213,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100067e,
        name_start: 16224,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000686,
        name_start: 16234,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000688,
        name_start: 16246,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000691,
        name_start: 16257,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000698,
        name_start: 16268,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006a4,
        name_start: 16278,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006a9,
        name_start: 16288,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006af,
        name_start: 16300,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006ba,
        name_start: 16310,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006be,
        name_start: 16328,
        name_len: 22,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006c1,
        name_start: 16350,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006cc,
        name_start: 16365,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000625,
        name_start: 16374,
        name_len: 16,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006d2,
        name_start: 16390,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006d4,
        name_start: 16406,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f0,
        name_start: 16421,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f1,
        name_start: 16428,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f2,
        name_start: 16435,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f3,
        name_start: 16442,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f4,
        name_start: 16449,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f5,
        name_start: 16456,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f6,
        name_start: 16463,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f7,
        name_start: 16470,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f8,
        name_start: 16477,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f9,
        name_start: 16484,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d82,
        name_start: 16491,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d83,
        name_start: 16498,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d85,
        name_start: 16505,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d86,
        name_start: 16511,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d87,
        name_start: 16518,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d88,
        name_start: 16525,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d89,
        name_start: 16533,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8a,
        name_start: 16539,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8b,
        name_start: 16546,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8c,
        name_start: 16552,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8d,
        name_start: 16559,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8e,
        name_start: 16566,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8f,
        name_start: 16574,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d90,
        name_start: 16581,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d91,
        name_start: 16589,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d92,
        name_start: 16595,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d93,
        name_start: 16602,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d94,
        name_start: 16609,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d95,
        name_start: 16615,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d96,
        name_start: 16622,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9a,
        name_start: 16629,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9b,
        name_start: 16636,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9c,
        name_start: 16644,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9d,
        name_start: 16651,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9e,
        name_start: 16659,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9f,
        name_start: 16667,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da0,
        name_start: 16675,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da1,
        name_start: 16682,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da2,
        name_start: 16690,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da3,
        name_start: 16697,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da4,
        name_start: 16705,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da5,
        name_start: 16713,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da6,
        name_start: 16722,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da7,
        name_start: 16730,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da8,
        name_start: 16738,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da9,
        name_start: 16747,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000daa,
        name_start: 16755,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dab,
        name_start: 16764,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dac,
        name_start: 16772,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dad,
        name_start: 16781,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dae,
        name_start: 16789,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000daf,
        name_start: 16798,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db0,
        name_start: 16806,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db1,
        name_start: 16815,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db3,
        name_start: 16822,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db4,
        name_start: 16831,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db5,
        name_start: 16838,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db6,
        name_start: 16846,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db7,
        name_start: 16853,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db8,
        name_start: 16861,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db9,
        name_start: 16868,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dba,
        name_start: 16876,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dbb,
        name_start: 16883,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dbd,
        name_start: 16890,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc0,
        name_start: 16897,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc1,
        name_start: 16904,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc2,
        name_start: 16912,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc3,
        name_start: 16921,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc4,
        name_start: 16928,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc5,
        name_start: 16935,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc6,
        name_start: 16943,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dca,
        name_start: 16950,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dcf,
        name_start: 16957,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd0,
        name_start: 16965,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd1,
        name_start: 16973,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd2,
        name_start: 16982,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd3,
        name_start: 16989,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd4,
        name_start: 16997,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd6,
        name_start: 17004,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd8,
        name_start: 17012,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd9,
        name_start: 17020,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dda,
        name_start: 17027,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddb,
        name_start: 17035,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddc,
        name_start: 17043,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddd,
        name_start: 17050,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dde,
        name_start: 17058,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddf,
        name_start: 17066,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000df2,
        name_start: 17074,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000df3,
        name_start: 17083,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000df4,
        name_start: 17092,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d0,
        name_start: 17107,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d1,
        name_start: 17118,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d2,
        name_start: 17130,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d3,
        name_start: 17142,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d4,
        name_start: 17154,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d5,
        name_start: 17165,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d6,
        name_start: 17177,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d7,
        name_start: 17189,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d8,
        name_start: 17201,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d9,
        name_start: 17212,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010da,
        name_start: 17224,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010db,
        name_start: 17236,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010dc,
        name_start: 17248,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010dd,
        name_start: 17260,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010de,
        name_start: 17271,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010df,
        name_start: 17283,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e0,
        name_start: 17296,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e1,
        name_start: 17308,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e2,
        name_start: 17320,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e3,
        name_start: 17332,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e4,
        name_start: 17343,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e5,
        name_start: 17356,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e6,
        name_start: 17369,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e7,
        name_start: 17382,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e8,
        name_start: 17394,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e9,
        name_start: 17407,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ea,
        name_start: 17420,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010eb,
        name_start: 17432,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ec,
        name_start: 17444,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ed,
        name_start: 17456,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ee,
        name_start: 17469,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ef,
        name_start: 17481,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f0,
        name_start: 17494,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f1,
        name_start: 17506,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f2,
        name_start: 17517,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f3,
        name_start: 17529,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f4,
        name_start: 17540,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f5,
        name_start: 17552,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f6,
        name_start: 17564,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e02,
        name_start: 17575,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e03,
        name_start: 17584,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e0a,
        name_start: 17593,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e0b,
        name_start: 17602,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e1e,
        name_start: 17611,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e1f,
        name_start: 17620,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e36,
        name_start: 17629,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e37,
        name_start: 17638,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e40,
        name_start: 17647,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e41,
        name_start: 17656,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e56,
        name_start: 17665,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e57,
        name_start: 17674,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e60,
        name_start: 17683,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e61,
        name_start: 17692,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e6a,
        name_start: 17701,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e6b,
        name_start: 17710,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e80,
        name_start: 17719,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e81,
        name_start: 17725,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e82,
        name_start: 17731,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e83,
        name_start: 17737,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e84,
        name_start: 17743,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e85,
        name_start: 17753,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e8a,
        name_start: 17763,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e8b,
        name_start: 17772,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea0,
        name_start: 17781,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea1,
        name_start: 17790,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea2,
        name_start: 17799,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea3,
        name_start: 17804,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea4,
        name_start: 17809,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea5,
        name_start: 17825,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea6,
        name_start: 17841,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea7,
        name_start: 17857,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea8,
        name_start: 17873,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea9,
        name_start: 17888,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eaa,
        name_start: 17903,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eab,
        name_start: 17919,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eac,
        name_start: 17935,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ead,
        name_start: 17954,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eae,
        name_start: 17973,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eaf,
        name_start: 17984,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb0,
        name_start: 17995,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb1,
        name_start: 18006,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb2,
        name_start: 18017,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb3,
        name_start: 18027,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb4,
        name_start: 18037,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb5,
        name_start: 18048,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb6,
        name_start: 18059,
        name_len: 14,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb7,
        name_start: 18073,
        name_len: 14,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb8,
        name_start: 18087,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb9,
        name_start: 18096,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eba,
        name_start: 18105,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebb,
        name_start: 18110,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebc,
        name_start: 18115,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebd,
        name_start: 18121,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebe,
        name_start: 18127,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebf,
        name_start: 18143,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec0,
        name_start: 18159,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec1,
        name_start: 18175,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec2,
        name_start: 18191,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec3,
        name_start: 18206,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec4,
        name_start: 18221,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec5,
        name_start: 18237,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec6,
        name_start: 18253,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec7,
        name_start: 18272,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec8,
        name_start: 18291,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec9,
        name_start: 18296,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eca,
        name_start: 18301,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecb,
        name_start: 18310,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecc,
        name_start: 18319,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecd,
        name_start: 18328,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ece,
        name_start: 18337,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecf,
        name_start: 18342,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed0,
        name_start: 18347,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed1,
        name_start: 18363,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed2,
        name_start: 18379,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed3,
        name_start: 18395,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed4,
        name_start: 18411,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed5,
        name_start: 18426,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed6,
        name_start: 18441,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed7,
        name_start: 18457,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed8,
        name_start: 18473,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed9,
        name_start: 18492,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eda,
        name_start: 18511,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edb,
        name_start: 18521,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edc,
        name_start: 18531,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edd,
        name_start: 18541,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ede,
        name_start: 18551,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edf,
        name_start: 18560,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee0,
        name_start: 18569,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee1,
        name_start: 18579,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee2,
        name_start: 18589,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee3,
        name_start: 18602,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee4,
        name_start: 18615,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee5,
        name_start: 18624,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee6,
        name_start: 18633,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee7,
        name_start: 18638,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee8,
        name_start: 18643,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee9,
        name_start: 18653,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eea,
        name_start: 18663,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eeb,
        name_start: 18673,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eec,
        name_start: 18683,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eed,
        name_start: 18692,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eee,
        name_start: 18701,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eef,
        name_start: 18711,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef0,
        name_start: 18721,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef1,
        name_start: 18734,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef2,
        name_start: 18747,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef3,
        name_start: 18753,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef4,
        name_start: 18759,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef5,
        name_start: 18768,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef6,
        name_start: 18777,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef7,
        name_start: 18782,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef8,
        name_start: 18787,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef9,
        name_start: 18793,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002070,
        name_start: 18799,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002074,
        name_start: 18811,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002075,
        name_start: 18823,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002076,
        name_start: 18835,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002077,
        name_start: 18846,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002078,
        name_start: 18859,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002079,
        name_start: 18872,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002080,
        name_start: 18884,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002081,
        name_start: 18897,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002082,
        name_start: 18909,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002083,
        name_start: 18921,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002084,
        name_start: 18935,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002085,
        name_start: 18948,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002086,
        name_start: 18961,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002087,
        name_start: 18973,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002088,
        name_start: 18987,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002089,
        name_start: 19001,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a0,
        name_start: 19014,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a1,
        name_start: 19021,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a2,
        name_start: 19030,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a3,
        name_start: 19042,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a4,
        name_start: 19052,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a5,
        name_start: 19060,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a6,
        name_start: 19068,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a7,
        name_start: 19077,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a8,
        name_start: 19087,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a9,
        name_start: 19096,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020aa,
        name_start: 19103,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020ab,
        name_start: 19116,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002202,
        name_start: 19124,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002205,
        name_start: 19140,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002208,
        name_start: 19148,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002209,
        name_start: 19157,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100220b,
        name_start: 19169,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100221a,
        name_start: 19179,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100221b,
        name_start: 19189,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100221c,
        name_start: 19197,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100222c,
        name_start: 19207,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100222d,
        name_start: 19216,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002235,
        name_start: 19225,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002247,
        name_start: 19232,
        name_len: 11,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002248,
        name_start: 19243,
        name_len: 8,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002262,
        name_start: 19251,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002263,
        name_start: 19263,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002800,
        name_start: 19271,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002801,
        name_start: 19284,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002802,
        name_start: 19298,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002803,
        name_start: 19312,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002804,
        name_start: 19327,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002805,
        name_start: 19341,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002806,
        name_start: 19356,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002807,
        name_start: 19371,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002808,
        name_start: 19387,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002809,
        name_start: 19401,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280a,
        name_start: 19416,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280b,
        name_start: 19431,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280c,
        name_start: 19447,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280d,
        name_start: 19462,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280e,
        name_start: 19478,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280f,
        name_start: 19494,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002810,
        name_start: 19511,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002811,
        name_start: 19525,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002812,
        name_start: 19540,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002813,
        name_start: 19555,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002814,
        name_start: 19571,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002815,
        name_start: 19586,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002816,
        name_start: 19602,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002817,
        name_start: 19618,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002818,
        name_start: 19635,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002819,
        name_start: 19650,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281a,
        name_start: 19666,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281b,
        name_start: 19682,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281c,
        name_start: 19699,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281d,
        name_start: 19715,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281e,
        name_start: 19732,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281f,
        name_start: 19749,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002820,
        name_start: 19767,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002821,
        name_start: 19781,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002822,
        name_start: 19796,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002823,
        name_start: 19811,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002824,
        name_start: 19827,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002825,
        name_start: 19842,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002826,
        name_start: 19858,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002827,
        name_start: 19874,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002828,
        name_start: 19891,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002829,
        name_start: 19906,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282a,
        name_start: 19922,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282b,
        name_start: 19938,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282c,
        name_start: 19955,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282d,
        name_start: 19971,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282e,
        name_start: 19988,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282f,
        name_start: 20005,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002830,
        name_start: 20023,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002831,
        name_start: 20038,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002832,
        name_start: 20054,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002833,
        name_start: 20070,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002834,
        name_start: 20087,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002835,
        name_start: 20103,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002836,
        name_start: 20120,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002837,
        name_start: 20137,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002838,
        name_start: 20155,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002839,
        name_start: 20171,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283a,
        name_start: 20188,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283b,
        name_start: 20205,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283c,
        name_start: 20223,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283d,
        name_start: 20240,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283e,
        name_start: 20258,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283f,
        name_start: 20276,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002840,
        name_start: 20295,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002841,
        name_start: 20309,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002842,
        name_start: 20324,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002843,
        name_start: 20339,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002844,
        name_start: 20355,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002845,
        name_start: 20370,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002846,
        name_start: 20386,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002847,
        name_start: 20402,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002848,
        name_start: 20419,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002849,
        name_start: 20434,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284a,
        name_start: 20450,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284b,
        name_start: 20466,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284c,
        name_start: 20483,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284d,
        name_start: 20499,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284e,
        name_start: 20516,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284f,
        name_start: 20533,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002850,
        name_start: 20551,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002851,
        name_start: 20566,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002852,
        name_start: 20582,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002853,
        name_start: 20598,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002854,
        name_start: 20615,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002855,
        name_start: 20631,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002856,
        name_start: 20648,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002857,
        name_start: 20665,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002858,
        name_start: 20683,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002859,
        name_start: 20699,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285a,
        name_start: 20716,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285b,
        name_start: 20733,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285c,
        name_start: 20751,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285d,
        name_start: 20768,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285e,
        name_start: 20786,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285f,
        name_start: 20804,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002860,
        name_start: 20823,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002861,
        name_start: 20838,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002862,
        name_start: 20854,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002863,
        name_start: 20870,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002864,
        name_start: 20887,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002865,
        name_start: 20903,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002866,
        name_start: 20920,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002867,
        name_start: 20937,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002868,
        name_start: 20955,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002869,
        name_start: 20971,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286a,
        name_start: 20988,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286b,
        name_start: 21005,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286c,
        name_start: 21023,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286d,
        name_start: 21040,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286e,
        name_start: 21058,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286f,
        name_start: 21076,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002870,
        name_start: 21095,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002871,
        name_start: 21111,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002872,
        name_start: 21128,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002873,
        name_start: 21145,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002874,
        name_start: 21163,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002875,
        name_start: 21180,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002876,
        name_start: 21198,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002877,
        name_start: 21216,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002878,
        name_start: 21235,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002879,
        name_start: 21252,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287a,
        name_start: 21270,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287b,
        name_start: 21288,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287c,
        name_start: 21307,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287d,
        name_start: 21325,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287e,
        name_start: 21344,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287f,
        name_start: 21363,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002880,
        name_start: 21383,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002881,
        name_start: 21397,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002882,
        name_start: 21412,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002883,
        name_start: 21427,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002884,
        name_start: 21443,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002885,
        name_start: 21458,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002886,
        name_start: 21474,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002887,
        name_start: 21490,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002888,
        name_start: 21507,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002889,
        name_start: 21522,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288a,
        name_start: 21538,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288b,
        name_start: 21554,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288c,
        name_start: 21571,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288d,
        name_start: 21587,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288e,
        name_start: 21604,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288f,
        name_start: 21621,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002890,
        name_start: 21639,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002891,
        name_start: 21654,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002892,
        name_start: 21670,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002893,
        name_start: 21686,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002894,
        name_start: 21703,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002895,
        name_start: 21719,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002896,
        name_start: 21736,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002897,
        name_start: 21753,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002898,
        name_start: 21771,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002899,
        name_start: 21787,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289a,
        name_start: 21804,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289b,
        name_start: 21821,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289c,
        name_start: 21839,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289d,
        name_start: 21856,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289e,
        name_start: 21874,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289f,
        name_start: 21892,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a0,
        name_start: 21911,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a1,
        name_start: 21926,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a2,
        name_start: 21942,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a3,
        name_start: 21958,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a4,
        name_start: 21975,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a5,
        name_start: 21991,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a6,
        name_start: 22008,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a7,
        name_start: 22025,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a8,
        name_start: 22043,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a9,
        name_start: 22059,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028aa,
        name_start: 22076,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ab,
        name_start: 22093,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ac,
        name_start: 22111,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ad,
        name_start: 22128,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ae,
        name_start: 22146,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028af,
        name_start: 22164,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b0,
        name_start: 22183,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b1,
        name_start: 22199,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b2,
        name_start: 22216,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b3,
        name_start: 22233,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b4,
        name_start: 22251,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b5,
        name_start: 22268,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b6,
        name_start: 22286,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b7,
        name_start: 22304,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b8,
        name_start: 22323,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b9,
        name_start: 22340,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ba,
        name_start: 22358,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bb,
        name_start: 22376,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bc,
        name_start: 22395,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bd,
        name_start: 22413,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028be,
        name_start: 22432,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bf,
        name_start: 22451,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c0,
        name_start: 22471,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c1,
        name_start: 22486,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c2,
        name_start: 22502,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c3,
        name_start: 22518,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c4,
        name_start: 22535,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c5,
        name_start: 22551,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c6,
        name_start: 22568,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c7,
        name_start: 22585,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c8,
        name_start: 22603,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c9,
        name_start: 22619,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ca,
        name_start: 22636,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cb,
        name_start: 22653,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cc,
        name_start: 22671,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cd,
        name_start: 22688,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ce,
        name_start: 22706,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cf,
        name_start: 22724,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d0,
        name_start: 22743,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d1,
        name_start: 22759,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d2,
        name_start: 22776,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d3,
        name_start: 22793,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d4,
        name_start: 22811,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d5,
        name_start: 22828,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d6,
        name_start: 22846,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d7,
        name_start: 22864,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d8,
        name_start: 22883,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d9,
        name_start: 22900,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028da,
        name_start: 22918,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028db,
        name_start: 22936,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028dc,
        name_start: 22955,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028dd,
        name_start: 22973,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028de,
        name_start: 22992,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028df,
        name_start: 23011,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e0,
        name_start: 23031,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e1,
        name_start: 23047,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e2,
        name_start: 23064,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e3,
        name_start: 23081,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e4,
        name_start: 23099,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e5,
        name_start: 23116,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e6,
        name_start: 23134,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e7,
        name_start: 23152,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e8,
        name_start: 23171,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e9,
        name_start: 23188,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ea,
        name_start: 23206,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028eb,
        name_start: 23224,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ec,
        name_start: 23243,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ed,
        name_start: 23261,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ee,
        name_start: 23280,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ef,
        name_start: 23299,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f0,
        name_start: 23319,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f1,
        name_start: 23336,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f2,
        name_start: 23354,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f3,
        name_start: 23372,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f4,
        name_start: 23391,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f5,
        name_start: 23409,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f6,
        name_start: 23428,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f7,
        name_start: 23447,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f8,
        name_start: 23467,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f9,
        name_start: 23485,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fa,
        name_start: 23504,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fb,
        name_start: 23523,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fc,
        name_start: 23543,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fd,
        name_start: 23562,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fe,
        name_start: 23582,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ff,
        name_start: 23602,
        name_len: 21,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000a8,
        name_start: 23623,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000848,
        name_start: 23635,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000a9,
        name_start: 23645,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000084a,
        name_start: 23657,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000aa,
        name_start: 23667,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000084c,
        name_start: 23685,
        name_len: 16,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000ab,
        name_start: 23701,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000084e,
        name_start: 23717,
        name_len: 14,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000ac,
        name_start: 23731,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000850,
        name_start: 23748,
        name_len: 15,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000af,
        name_start: 23763,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000852,
        name_start: 23769,
        name_len: 4,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000be,
        name_start: 23773,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000854,
        name_start: 23782,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000ee,
        name_start: 23789,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000856,
        name_start: 23801,
        name_len: 4,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000856,
        name_start: 23805,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000f6,
        name_start: 23807,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000859,
        name_start: 23818,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000fc,
        name_start: 23827,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000085b,
        name_start: 23834,
        name_len: 5,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe22,
        name_start: 23839,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe27,
        name_start: 23849,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe2c,
        name_start: 23862,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe5e,
        name_start: 23877,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe60,
        name_start: 23895,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe7e,
        name_start: 23908,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000feb0,
        name_start: 23914,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff00,
        name_start: 23926,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff48,
        name_start: 23933,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff49,
        name_start: 23944,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6c,
        name_start: 23955,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000867,
        name_start: 23962,
        name_len: 5,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6d,
        name_start: 23967,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000869,
        name_start: 23975,
        name_len: 6,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6e,
        name_start: 23981,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000086b,
        name_start: 23987,
        name_len: 4,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6f,
        name_start: 23991,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000086d,
        name_start: 24002,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff70,
        name_start: 24011,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000086f,
        name_start: 24023,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff71,
        name_start: 24033,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000871,
        name_start: 24045,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff72,
        name_start: 24055,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000873,
        name_start: 24067,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff73,
        name_start: 24077,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000875,
        name_start: 24089,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff74,
        name_start: 24099,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000877,
        name_start: 24108,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff75,
        name_start: 24115,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000879,
        name_start: 24127,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff76,
        name_start: 24137,
        name_len: 10,
        flags: 0 | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff77,
        name_start: 24147,
        name_len: 10,
        flags: 0 | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff02,
        name_start: 24157,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff03,
        name_start: 24164,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff04,
        name_start: 24170,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff07,
        name_start: 24178,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff08,
        name_start: 24188,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff0b,
        name_start: 24200,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff1b,
        name_start: 24208,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff31,
        name_start: 24217,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff32,
        name_start: 24227,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff33,
        name_start: 24242,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff40,
        name_start: 24255,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff41,
        name_start: 24266,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff42,
        name_start: 24275,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff43,
        name_start: 24286,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff44,
        name_start: 24298,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff45,
        name_start: 24309,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff51,
        name_start: 24319,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff52,
        name_start: 24326,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff53,
        name_start: 24331,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff54,
        name_start: 24339,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff57,
        name_start: 24346,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff58,
        name_start: 24356,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff59,
        name_start: 24368,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5a,
        name_start: 24378,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5b,
        name_start: 24390,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5c,
        name_start: 24401,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5d,
        name_start: 24412,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5e,
        name_start: 24424,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff60,
        name_start: 24436,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff63,
        name_start: 24445,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff65,
        name_start: 24454,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff67,
        name_start: 24461,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff69,
        name_start: 24468,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff6a,
        name_start: 24477,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff71,
        name_start: 24484,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff72,
        name_start: 24496,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff73,
        name_start: 24510,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff74,
        name_start: 24521,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff78,
        name_start: 24530,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ffff,
        name_start: 24540,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff00,
        name_start: 24549,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff01,
        name_start: 24560,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff02,
        name_start: 24572,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff03,
        name_start: 24583,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff04,
        name_start: 24594,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff05,
        name_start: 24609,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff10,
        name_start: 24622,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff11,
        name_start: 24628,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff60,
        name_start: 24634,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff70,
        name_start: 24644,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff71,
        name_start: 24652,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff72,
        name_start: 24660,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff73,
        name_start: 24667,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff74,
        name_start: 24674,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff75,
        name_start: 24682,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff76,
        name_start: 24688,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff77,
        name_start: 24702,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff78,
        name_start: 24721,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff79,
        name_start: 24733,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7a,
        name_start: 24752,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7b,
        name_start: 24767,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7c,
        name_start: 24790,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7d,
        name_start: 24813,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810a4,
        name_start: 24832,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810ae,
        name_start: 24850,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810d1,
        name_start: 24858,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810dc,
        name_start: 24876,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810f4,
        name_start: 24885,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c0,
        name_start: 24903,
        name_len: 21,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810f5,
        name_start: 24924,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081160,
        name_start: 24938,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081162,
        name_start: 24944,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081166,
        name_start: 24952,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081168,
        name_start: 24960,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008116a,
        name_start: 24974,
        name_len: 27,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008116e,
        name_start: 25001,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081170,
        name_start: 25020,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081171,
        name_start: 25041,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081175,
        name_start: 25059,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081177,
        name_start: 25079,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081178,
        name_start: 25094,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081179,
        name_start: 25111,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008117a,
        name_start: 25128,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008117b,
        name_start: 25148,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008117c,
        name_start: 25166,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008117d,
        name_start: 25188,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081180,
        name_start: 25212,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081181,
        name_start: 25231,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081182,
        name_start: 25251,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081183,
        name_start: 25271,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081184,
        name_start: 25286,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081185,
        name_start: 25309,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008d8,
        name_start: 25316,
        name_len: 18,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081186,
        name_start: 25334,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081188,
        name_start: 25358,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081192,
        name_start: 25367,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081193,
        name_start: 25380,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081199,
        name_start: 25395,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008119b,
        name_start: 25412,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008119d,
        name_start: 25421,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a0,
        name_start: 25440,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a4,
        name_start: 25454,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a6,
        name_start: 25467,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a8,
        name_start: 25477,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a9,
        name_start: 25495,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811aa,
        name_start: 25511,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811ac,
        name_start: 25523,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811ad,
        name_start: 25536,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811af,
        name_start: 25551,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b0,
        name_start: 25568,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b6,
        name_start: 25582,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b7,
        name_start: 25597,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b8,
        name_start: 25612,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b9,
        name_start: 25628,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811ba,
        name_start: 25646,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811bc,
        name_start: 25656,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811bd,
        name_start: 25678,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811be,
        name_start: 25693,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811bf,
        name_start: 25708,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d0,
        name_start: 25721,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d1,
        name_start: 25727,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d2,
        name_start: 25737,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d3,
        name_start: 25746,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d4,
        name_start: 25755,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d5,
        name_start: 25764,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d6,
        name_start: 25773,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d7,
        name_start: 25782,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d8,
        name_start: 25791,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d9,
        name_start: 25800,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811da,
        name_start: 25809,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811db,
        name_start: 25818,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811dc,
        name_start: 25828,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811dd,
        name_start: 25838,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811de,
        name_start: 25848,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811df,
        name_start: 25856,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e0,
        name_start: 25864,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e1,
        name_start: 25872,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e2,
        name_start: 25880,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e3,
        name_start: 25888,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e4,
        name_start: 25896,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e5,
        name_start: 25904,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081200,
        name_start: 25920,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081201,
        name_start: 25932,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081202,
        name_start: 25944,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081203,
        name_start: 25956,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081204,
        name_start: 25968,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081205,
        name_start: 25980,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081206,
        name_start: 25992,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081207,
        name_start: 26004,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081208,
        name_start: 26016,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081209,
        name_start: 26028,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120a,
        name_start: 26040,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120b,
        name_start: 26055,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120c,
        name_start: 26071,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120d,
        name_start: 26083,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120e,
        name_start: 26095,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120f,
        name_start: 26107,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081210,
        name_start: 26119,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081211,
        name_start: 26134,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081215,
        name_start: 26147,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081216,
        name_start: 26163,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081217,
        name_start: 26180,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081218,
        name_start: 26192,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081219,
        name_start: 26206,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121a,
        name_start: 26220,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121b,
        name_start: 26235,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121c,
        name_start: 26250,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121d,
        name_start: 26266,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121e,
        name_start: 26285,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081230,
        name_start: 26301,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081232,
        name_start: 26314,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081240,
        name_start: 26335,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081241,
        name_start: 26351,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081242,
        name_start: 26366,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081243,
        name_start: 26377,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081244,
        name_start: 26393,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081245,
        name_start: 26406,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081246,
        name_start: 26421,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081247,
        name_start: 26437,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081249,
        name_start: 26450,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124a,
        name_start: 26465,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124b,
        name_start: 26476,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124c,
        name_start: 26498,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124d,
        name_start: 26521,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124e,
        name_start: 26543,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124f,
        name_start: 26560,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081250,
        name_start: 26576,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081251,
        name_start: 26593,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081260,
        name_start: 26610,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081261,
        name_start: 26632,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081262,
        name_start: 26654,
        name_len: 27,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081263,
        name_start: 26681,
        name_len: 27,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081264,
        name_start: 26708,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081265,
        name_start: 26732,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081266,
        name_start: 26756,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081267,
        name_start: 26767,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081268,
        name_start: 26780,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081269,
        name_start: 26790,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126a,
        name_start: 26802,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126b,
        name_start: 26814,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126c,
        name_start: 26830,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126d,
        name_start: 26843,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126e,
        name_start: 26856,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126f,
        name_start: 26869,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081270,
        name_start: 26879,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081271,
        name_start: 26895,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081272,
        name_start: 26909,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081273,
        name_start: 26924,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081274,
        name_start: 26931,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081275,
        name_start: 26941,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081276,
        name_start: 26956,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081277,
        name_start: 26971,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081278,
        name_start: 26979,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081279,
        name_start: 26999,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127a,
        name_start: 27022,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127b,
        name_start: 27045,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127c,
        name_start: 27060,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127d,
        name_start: 27079,
        name_len: 25,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127e,
        name_start: 27104,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127f,
        name_start: 27120,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081280,
        name_start: 27127,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081281,
        name_start: 27139,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081282,
        name_start: 27155,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081283,
        name_start: 27175,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081284,
        name_start: 27193,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081285,
        name_start: 27209,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081286,
        name_start: 27229,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081287,
        name_start: 27245,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081288,
        name_start: 27260,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081290,
        name_start: 27271,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081291,
        name_start: 27281,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081292,
        name_start: 27291,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081293,
        name_start: 27301,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081294,
        name_start: 27311,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081295,
        name_start: 27321,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081296,
        name_start: 27331,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081297,
        name_start: 27341,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081298,
        name_start: 27351,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081299,
        name_start: 27361,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129a,
        name_start: 27372,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129b,
        name_start: 27383,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129c,
        name_start: 27394,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129d,
        name_start: 27405,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129e,
        name_start: 27416,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129f,
        name_start: 27427,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a0,
        name_start: 27438,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a1,
        name_start: 27449,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a2,
        name_start: 27460,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a3,
        name_start: 27471,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a4,
        name_start: 27482,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a5,
        name_start: 27493,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a6,
        name_start: 27504,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a7,
        name_start: 27515,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a8,
        name_start: 27526,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a9,
        name_start: 27537,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812aa,
        name_start: 27548,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ab,
        name_start: 27559,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ac,
        name_start: 27570,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ad,
        name_start: 27581,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b0,
        name_start: 27592,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b1,
        name_start: 27612,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b2,
        name_start: 27631,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b3,
        name_start: 27651,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b4,
        name_start: 27667,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b5,
        name_start: 27683,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b8,
        name_start: 27699,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b9,
        name_start: 27714,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ba,
        name_start: 27729,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812bb,
        name_start: 27744,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812bc,
        name_start: 27759,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812bd,
        name_start: 27774,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe01,
        name_start: 27793,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe02,
        name_start: 27808,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe03,
        name_start: 27823,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe04,
        name_start: 27838,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe05,
        name_start: 27853,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe06,
        name_start: 27868,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe07,
        name_start: 27883,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe08,
        name_start: 27898,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe09,
        name_start: 27913,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe0a,
        name_start: 27928,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe0b,
        name_start: 27944,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe0c,
        name_start: 27960,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe20,
        name_start: 27976,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe21,
        name_start: 27986,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe22,
        name_start: 27999,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe23,
        name_start: 28013,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe24,
        name_start: 28027,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe25,
        name_start: 28044,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff01,
        name_start: 28059,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff02,
        name_start: 28071,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff03,
        name_start: 28090,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff04,
        name_start: 28111,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff05,
        name_start: 28128,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff06,
        name_start: 28147,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff07,
        name_start: 28168,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff10,
        name_start: 28190,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff11,
        name_start: 28201,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff12,
        name_start: 28221,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff13,
        name_start: 28234,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff14,
        name_start: 28254,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff15,
        name_start: 28267,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff16,
        name_start: 28280,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff17,
        name_start: 28293,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff18,
        name_start: 28306,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff19,
        name_start: 28318,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1a,
        name_start: 28326,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1b,
        name_start: 28335,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1c,
        name_start: 28345,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1d,
        name_start: 28360,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1e,
        name_start: 28374,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1f,
        name_start: 28382,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff20,
        name_start: 28394,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff21,
        name_start: 28406,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff22,
        name_start: 28419,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff23,
        name_start: 28437,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff24,
        name_start: 28449,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff25,
        name_start: 28463,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff26,
        name_start: 28478,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff27,
        name_start: 28486,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff28,
        name_start: 28497,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff29,
        name_start: 28505,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2a,
        name_start: 28516,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2b,
        name_start: 28528,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2c,
        name_start: 28538,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2d,
        name_start: 28547,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2e,
        name_start: 28562,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2f,
        name_start: 28569,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff30,
        name_start: 28578,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff31,
        name_start: 28591,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff32,
        name_start: 28605,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff33,
        name_start: 28619,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff34,
        name_start: 28633,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff35,
        name_start: 28647,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff36,
        name_start: 28660,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff37,
        name_start: 28668,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff38,
        name_start: 28679,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff39,
        name_start: 28690,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3a,
        name_start: 28705,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3b,
        name_start: 28717,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3c,
        name_start: 28737,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3d,
        name_start: 28748,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3e,
        name_start: 28761,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3f,
        name_start: 28776,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff40,
        name_start: 28791,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff41,
        name_start: 28802,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff42,
        name_start: 28813,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff43,
        name_start: 28824,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff44,
        name_start: 28835,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff45,
        name_start: 28846,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff46,
        name_start: 28857,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff47,
        name_start: 28868,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff48,
        name_start: 28879,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff49,
        name_start: 28890,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4a,
        name_start: 28901,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4b,
        name_start: 28912,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4c,
        name_start: 28923,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4d,
        name_start: 28934,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4e,
        name_start: 28945,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4f,
        name_start: 28956,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff50,
        name_start: 28967,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff51,
        name_start: 28986,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff52,
        name_start: 29006,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff53,
        name_start: 29014,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e8,
        name_start: 29020,
        name_len: 17,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff54,
        name_start: 29037,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff55,
        name_start: 29051,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff56,
        name_start: 29060,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff57,
        name_start: 29069,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff58,
        name_start: 29077,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff59,
        name_start: 29084,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5a,
        name_start: 29095,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5b,
        name_start: 29102,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5c,
        name_start: 29115,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5d,
        name_start: 29124,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5e,
        name_start: 29136,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5f,
        name_start: 29144,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff60,
        name_start: 29150,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff61,
        name_start: 29160,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff62,
        name_start: 29170,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff63,
        name_start: 29180,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff65,
        name_start: 29191,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff66,
        name_start: 29201,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff67,
        name_start: 29211,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff68,
        name_start: 29222,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff69,
        name_start: 29229,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6a,
        name_start: 29237,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6b,
        name_start: 29251,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6c,
        name_start: 29259,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6d,
        name_start: 29269,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6e,
        name_start: 29278,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff70,
        name_start: 29287,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff72,
        name_start: 29292,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff73,
        name_start: 29301,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff74,
        name_start: 29311,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff75,
        name_start: 29328,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff76,
        name_start: 29342,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff77,
        name_start: 29356,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff78,
        name_start: 29364,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff79,
        name_start: 29376,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7a,
        name_start: 29390,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7b,
        name_start: 29405,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7c,
        name_start: 29413,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7d,
        name_start: 29422,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7e,
        name_start: 29437,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7f,
        name_start: 29448,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff80,
        name_start: 29460,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff81,
        name_start: 29472,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff82,
        name_start: 29481,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff84,
        name_start: 29491,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff85,
        name_start: 29501,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff86,
        name_start: 29512,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff87,
        name_start: 29523,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff88,
        name_start: 29532,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff89,
        name_start: 29547,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8a,
        name_start: 29555,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8b,
        name_start: 29563,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8c,
        name_start: 29573,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8d,
        name_start: 29584,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8e,
        name_start: 29592,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8f,
        name_start: 29605,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff90,
        name_start: 29615,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff91,
        name_start: 29630,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff92,
        name_start: 29642,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff93,
        name_start: 29651,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff94,
        name_start: 29662,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff95,
        name_start: 29675,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff96,
        name_start: 29683,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff97,
        name_start: 29690,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff98,
        name_start: 29706,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff99,
        name_start: 29721,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9a,
        name_start: 29740,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9b,
        name_start: 29752,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9c,
        name_start: 29771,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9d,
        name_start: 29785,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9e,
        name_start: 29798,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9f,
        name_start: 29814,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa0,
        name_start: 29822,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa1,
        name_start: 29832,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa2,
        name_start: 29840,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa3,
        name_start: 29851,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa4,
        name_start: 29858,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa5,
        name_start: 29867,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa6,
        name_start: 29877,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa7,
        name_start: 29885,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa8,
        name_start: 29896,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa9,
        name_start: 29909,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb0,
        name_start: 29927,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb1,
        name_start: 29941,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb2,
        name_start: 29956,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb3,
        name_start: 29972,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb4,
        name_start: 29984,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb5,
        name_start: 29992,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb6,
        name_start: 30002,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb7,
        name_start: 30017,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb8,
        name_start: 30039,
        name_len: 14,
        flags: 0,
    },
];

pub mod syms {
    #![allow(non_upper_case_globals)]
    use super::*;
    /// NoSymbol
    pub const NoSymbol: Keysym = Keysym(0x00000000);
    /// space
    pub const space: Keysym = Keysym(0x00000020);
    /// exclam
    pub const exclam: Keysym = Keysym(0x00000021);
    /// quotedbl
    pub const quotedbl: Keysym = Keysym(0x00000022);
    /// numbersign
    pub const numbersign: Keysym = Keysym(0x00000023);
    /// dollar
    pub const dollar: Keysym = Keysym(0x00000024);
    /// percent
    pub const percent: Keysym = Keysym(0x00000025);
    /// ampersand
    pub const ampersand: Keysym = Keysym(0x00000026);
    /// apostrophe
    pub const apostrophe: Keysym = Keysym(0x00000027);
    /// quoteright
    pub const quoteright: Keysym = Keysym(0x00000027);
    /// parenleft
    pub const parenleft: Keysym = Keysym(0x00000028);
    /// parenright
    pub const parenright: Keysym = Keysym(0x00000029);
    /// asterisk
    pub const asterisk: Keysym = Keysym(0x0000002a);
    /// plus
    pub const plus: Keysym = Keysym(0x0000002b);
    /// comma
    pub const comma: Keysym = Keysym(0x0000002c);
    /// minus
    pub const minus: Keysym = Keysym(0x0000002d);
    /// period
    pub const period: Keysym = Keysym(0x0000002e);
    /// slash
    pub const slash: Keysym = Keysym(0x0000002f);
    /// 0
    pub const _0: Keysym = Keysym(0x00000030);
    /// 1
    pub const _1: Keysym = Keysym(0x00000031);
    /// 2
    pub const _2: Keysym = Keysym(0x00000032);
    /// 3
    pub const _3: Keysym = Keysym(0x00000033);
    /// 4
    pub const _4: Keysym = Keysym(0x00000034);
    /// 5
    pub const _5: Keysym = Keysym(0x00000035);
    /// 6
    pub const _6: Keysym = Keysym(0x00000036);
    /// 7
    pub const _7: Keysym = Keysym(0x00000037);
    /// 8
    pub const _8: Keysym = Keysym(0x00000038);
    /// 9
    pub const _9: Keysym = Keysym(0x00000039);
    /// colon
    pub const colon: Keysym = Keysym(0x0000003a);
    /// semicolon
    pub const semicolon: Keysym = Keysym(0x0000003b);
    /// less
    pub const less: Keysym = Keysym(0x0000003c);
    /// equal
    pub const equal: Keysym = Keysym(0x0000003d);
    /// greater
    pub const greater: Keysym = Keysym(0x0000003e);
    /// question
    pub const question: Keysym = Keysym(0x0000003f);
    /// at
    pub const at: Keysym = Keysym(0x00000040);
    /// A
    pub const A: Keysym = Keysym(0x00000041);
    /// B
    pub const B: Keysym = Keysym(0x00000042);
    /// C
    pub const C: Keysym = Keysym(0x00000043);
    /// D
    pub const D: Keysym = Keysym(0x00000044);
    /// E
    pub const E: Keysym = Keysym(0x00000045);
    /// F
    pub const F: Keysym = Keysym(0x00000046);
    /// G
    pub const G: Keysym = Keysym(0x00000047);
    /// H
    pub const H: Keysym = Keysym(0x00000048);
    /// I
    pub const I: Keysym = Keysym(0x00000049);
    /// J
    pub const J: Keysym = Keysym(0x0000004a);
    /// K
    pub const K: Keysym = Keysym(0x0000004b);
    /// L
    pub const L: Keysym = Keysym(0x0000004c);
    /// M
    pub const M: Keysym = Keysym(0x0000004d);
    /// N
    pub const N: Keysym = Keysym(0x0000004e);
    /// O
    pub const O: Keysym = Keysym(0x0000004f);
    /// P
    pub const P: Keysym = Keysym(0x00000050);
    /// Q
    pub const Q: Keysym = Keysym(0x00000051);
    /// R
    pub const R: Keysym = Keysym(0x00000052);
    /// S
    pub const S: Keysym = Keysym(0x00000053);
    /// T
    pub const T: Keysym = Keysym(0x00000054);
    /// U
    pub const U: Keysym = Keysym(0x00000055);
    /// V
    pub const V: Keysym = Keysym(0x00000056);
    /// W
    pub const W: Keysym = Keysym(0x00000057);
    /// X
    pub const X: Keysym = Keysym(0x00000058);
    /// Y
    pub const Y: Keysym = Keysym(0x00000059);
    /// Z
    pub const Z: Keysym = Keysym(0x0000005a);
    /// bracketleft
    pub const bracketleft: Keysym = Keysym(0x0000005b);
    /// backslash
    pub const backslash: Keysym = Keysym(0x0000005c);
    /// bracketright
    pub const bracketright: Keysym = Keysym(0x0000005d);
    /// asciicircum
    pub const asciicircum: Keysym = Keysym(0x0000005e);
    /// underscore
    pub const underscore: Keysym = Keysym(0x0000005f);
    /// grave
    pub const grave: Keysym = Keysym(0x00000060);
    /// quoteleft
    pub const quoteleft: Keysym = Keysym(0x00000060);
    /// a
    pub const a: Keysym = Keysym(0x00000061);
    /// b
    pub const b: Keysym = Keysym(0x00000062);
    /// c
    pub const c: Keysym = Keysym(0x00000063);
    /// d
    pub const d: Keysym = Keysym(0x00000064);
    /// e
    pub const e: Keysym = Keysym(0x00000065);
    /// f
    pub const f: Keysym = Keysym(0x00000066);
    /// g
    pub const g: Keysym = Keysym(0x00000067);
    /// h
    pub const h: Keysym = Keysym(0x00000068);
    /// i
    pub const i: Keysym = Keysym(0x00000069);
    /// j
    pub const j: Keysym = Keysym(0x0000006a);
    /// k
    pub const k: Keysym = Keysym(0x0000006b);
    /// l
    pub const l: Keysym = Keysym(0x0000006c);
    /// m
    pub const m: Keysym = Keysym(0x0000006d);
    /// n
    pub const n: Keysym = Keysym(0x0000006e);
    /// o
    pub const o: Keysym = Keysym(0x0000006f);
    /// p
    pub const p: Keysym = Keysym(0x00000070);
    /// q
    pub const q: Keysym = Keysym(0x00000071);
    /// r
    pub const r: Keysym = Keysym(0x00000072);
    /// s
    pub const s: Keysym = Keysym(0x00000073);
    /// t
    pub const t: Keysym = Keysym(0x00000074);
    /// u
    pub const u: Keysym = Keysym(0x00000075);
    /// v
    pub const v: Keysym = Keysym(0x00000076);
    /// w
    pub const w: Keysym = Keysym(0x00000077);
    /// x
    pub const x: Keysym = Keysym(0x00000078);
    /// y
    pub const y: Keysym = Keysym(0x00000079);
    /// z
    pub const z: Keysym = Keysym(0x0000007a);
    /// braceleft
    pub const braceleft: Keysym = Keysym(0x0000007b);
    /// bar
    pub const bar: Keysym = Keysym(0x0000007c);
    /// braceright
    pub const braceright: Keysym = Keysym(0x0000007d);
    /// asciitilde
    pub const asciitilde: Keysym = Keysym(0x0000007e);
    /// nobreakspace
    pub const nobreakspace: Keysym = Keysym(0x000000a0);
    /// exclamdown
    pub const exclamdown: Keysym = Keysym(0x000000a1);
    /// cent
    pub const cent: Keysym = Keysym(0x000000a2);
    /// sterling
    pub const sterling: Keysym = Keysym(0x000000a3);
    /// currency
    pub const currency: Keysym = Keysym(0x000000a4);
    /// yen
    pub const yen: Keysym = Keysym(0x000000a5);
    /// brokenbar
    pub const brokenbar: Keysym = Keysym(0x000000a6);
    /// section
    pub const section: Keysym = Keysym(0x000000a7);
    /// diaeresis
    pub const diaeresis: Keysym = Keysym(0x000000a8);
    /// copyright
    pub const copyright: Keysym = Keysym(0x000000a9);
    /// ordfeminine
    pub const ordfeminine: Keysym = Keysym(0x000000aa);
    /// guillemotleft
    pub const guillemotleft: Keysym = Keysym(0x000000ab);
    /// guillemetleft
    pub const guillemetleft: Keysym = Keysym(0x000000ab);
    /// notsign
    pub const notsign: Keysym = Keysym(0x000000ac);
    /// hyphen
    pub const hyphen: Keysym = Keysym(0x000000ad);
    /// registered
    pub const registered: Keysym = Keysym(0x000000ae);
    /// macron
    pub const macron: Keysym = Keysym(0x000000af);
    /// degree
    pub const degree: Keysym = Keysym(0x000000b0);
    /// plusminus
    pub const plusminus: Keysym = Keysym(0x000000b1);
    /// twosuperior
    pub const twosuperior: Keysym = Keysym(0x000000b2);
    /// threesuperior
    pub const threesuperior: Keysym = Keysym(0x000000b3);
    /// acute
    pub const acute: Keysym = Keysym(0x000000b4);
    /// mu
    pub const mu: Keysym = Keysym(0x000000b5);
    /// paragraph
    pub const paragraph: Keysym = Keysym(0x000000b6);
    /// periodcentered
    pub const periodcentered: Keysym = Keysym(0x000000b7);
    /// cedilla
    pub const cedilla: Keysym = Keysym(0x000000b8);
    /// onesuperior
    pub const onesuperior: Keysym = Keysym(0x000000b9);
    /// masculine
    pub const masculine: Keysym = Keysym(0x000000ba);
    /// ordmasculine
    pub const ordmasculine: Keysym = Keysym(0x000000ba);
    /// guillemotright
    pub const guillemotright: Keysym = Keysym(0x000000bb);
    /// guillemetright
    pub const guillemetright: Keysym = Keysym(0x000000bb);
    /// onequarter
    pub const onequarter: Keysym = Keysym(0x000000bc);
    /// onehalf
    pub const onehalf: Keysym = Keysym(0x000000bd);
    /// threequarters
    pub const threequarters: Keysym = Keysym(0x000000be);
    /// questiondown
    pub const questiondown: Keysym = Keysym(0x000000bf);
    /// Agrave
    pub const Agrave: Keysym = Keysym(0x000000c0);
    /// Aacute
    pub const Aacute: Keysym = Keysym(0x000000c1);
    /// Acircumflex
    pub const Acircumflex: Keysym = Keysym(0x000000c2);
    /// Atilde
    pub const Atilde: Keysym = Keysym(0x000000c3);
    /// Adiaeresis
    pub const Adiaeresis: Keysym = Keysym(0x000000c4);
    /// Aring
    pub const Aring: Keysym = Keysym(0x000000c5);
    /// AE
    pub const AE: Keysym = Keysym(0x000000c6);
    /// Ccedilla
    pub const Ccedilla: Keysym = Keysym(0x000000c7);
    /// Egrave
    pub const Egrave: Keysym = Keysym(0x000000c8);
    /// Eacute
    pub const Eacute: Keysym = Keysym(0x000000c9);
    /// Ecircumflex
    pub const Ecircumflex: Keysym = Keysym(0x000000ca);
    /// Ediaeresis
    pub const Ediaeresis: Keysym = Keysym(0x000000cb);
    /// Igrave
    pub const Igrave: Keysym = Keysym(0x000000cc);
    /// Iacute
    pub const Iacute: Keysym = Keysym(0x000000cd);
    /// Icircumflex
    pub const Icircumflex: Keysym = Keysym(0x000000ce);
    /// Idiaeresis
    pub const Idiaeresis: Keysym = Keysym(0x000000cf);
    /// ETH
    pub const ETH: Keysym = Keysym(0x000000d0);
    /// Eth
    pub const Eth: Keysym = Keysym(0x000000d0);
    /// Ntilde
    pub const Ntilde: Keysym = Keysym(0x000000d1);
    /// Ograve
    pub const Ograve: Keysym = Keysym(0x000000d2);
    /// Oacute
    pub const Oacute: Keysym = Keysym(0x000000d3);
    /// Ocircumflex
    pub const Ocircumflex: Keysym = Keysym(0x000000d4);
    /// Otilde
    pub const Otilde: Keysym = Keysym(0x000000d5);
    /// Odiaeresis
    pub const Odiaeresis: Keysym = Keysym(0x000000d6);
    /// multiply
    pub const multiply: Keysym = Keysym(0x000000d7);
    /// Oslash
    pub const Oslash: Keysym = Keysym(0x000000d8);
    /// Ooblique
    pub const Ooblique: Keysym = Keysym(0x000000d8);
    /// Ugrave
    pub const Ugrave: Keysym = Keysym(0x000000d9);
    /// Uacute
    pub const Uacute: Keysym = Keysym(0x000000da);
    /// Ucircumflex
    pub const Ucircumflex: Keysym = Keysym(0x000000db);
    /// Udiaeresis
    pub const Udiaeresis: Keysym = Keysym(0x000000dc);
    /// Yacute
    pub const Yacute: Keysym = Keysym(0x000000dd);
    /// THORN
    pub const THORN: Keysym = Keysym(0x000000de);
    /// Thorn
    pub const Thorn: Keysym = Keysym(0x000000de);
    /// ssharp
    pub const ssharp: Keysym = Keysym(0x000000df);
    /// agrave
    pub const agrave: Keysym = Keysym(0x000000e0);
    /// aacute
    pub const aacute: Keysym = Keysym(0x000000e1);
    /// acircumflex
    pub const acircumflex: Keysym = Keysym(0x000000e2);
    /// atilde
    pub const atilde: Keysym = Keysym(0x000000e3);
    /// adiaeresis
    pub const adiaeresis: Keysym = Keysym(0x000000e4);
    /// aring
    pub const aring: Keysym = Keysym(0x000000e5);
    /// ae
    pub const ae: Keysym = Keysym(0x000000e6);
    /// ccedilla
    pub const ccedilla: Keysym = Keysym(0x000000e7);
    /// egrave
    pub const egrave: Keysym = Keysym(0x000000e8);
    /// eacute
    pub const eacute: Keysym = Keysym(0x000000e9);
    /// ecircumflex
    pub const ecircumflex: Keysym = Keysym(0x000000ea);
    /// ediaeresis
    pub const ediaeresis: Keysym = Keysym(0x000000eb);
    /// igrave
    pub const igrave: Keysym = Keysym(0x000000ec);
    /// iacute
    pub const iacute: Keysym = Keysym(0x000000ed);
    /// icircumflex
    pub const icircumflex: Keysym = Keysym(0x000000ee);
    /// idiaeresis
    pub const idiaeresis: Keysym = Keysym(0x000000ef);
    /// eth
    pub const eth: Keysym = Keysym(0x000000f0);
    /// ntilde
    pub const ntilde: Keysym = Keysym(0x000000f1);
    /// ograve
    pub const ograve: Keysym = Keysym(0x000000f2);
    /// oacute
    pub const oacute: Keysym = Keysym(0x000000f3);
    /// ocircumflex
    pub const ocircumflex: Keysym = Keysym(0x000000f4);
    /// otilde
    pub const otilde: Keysym = Keysym(0x000000f5);
    /// odiaeresis
    pub const odiaeresis: Keysym = Keysym(0x000000f6);
    /// division
    pub const division: Keysym = Keysym(0x000000f7);
    /// oslash
    pub const oslash: Keysym = Keysym(0x000000f8);
    /// ooblique
    pub const ooblique: Keysym = Keysym(0x000000f8);
    /// ugrave
    pub const ugrave: Keysym = Keysym(0x000000f9);
    /// uacute
    pub const uacute: Keysym = Keysym(0x000000fa);
    /// ucircumflex
    pub const ucircumflex: Keysym = Keysym(0x000000fb);
    /// udiaeresis
    pub const udiaeresis: Keysym = Keysym(0x000000fc);
    /// yacute
    pub const yacute: Keysym = Keysym(0x000000fd);
    /// thorn
    pub const thorn: Keysym = Keysym(0x000000fe);
    /// ydiaeresis
    pub const ydiaeresis: Keysym = Keysym(0x000000ff);
    /// Aogonek
    pub const Aogonek: Keysym = Keysym(0x000001a1);
    /// breve
    pub const breve: Keysym = Keysym(0x000001a2);
    /// Lstroke
    pub const Lstroke: Keysym = Keysym(0x000001a3);
    /// Lcaron
    pub const Lcaron: Keysym = Keysym(0x000001a5);
    /// Sacute
    pub const Sacute: Keysym = Keysym(0x000001a6);
    /// Scaron
    pub const Scaron: Keysym = Keysym(0x000001a9);
    /// Scedilla
    pub const Scedilla: Keysym = Keysym(0x000001aa);
    /// Tcaron
    pub const Tcaron: Keysym = Keysym(0x000001ab);
    /// Zacute
    pub const Zacute: Keysym = Keysym(0x000001ac);
    /// Zcaron
    pub const Zcaron: Keysym = Keysym(0x000001ae);
    /// Zabovedot
    pub const Zabovedot: Keysym = Keysym(0x000001af);
    /// aogonek
    pub const aogonek: Keysym = Keysym(0x000001b1);
    /// ogonek
    pub const ogonek: Keysym = Keysym(0x000001b2);
    /// lstroke
    pub const lstroke: Keysym = Keysym(0x000001b3);
    /// lcaron
    pub const lcaron: Keysym = Keysym(0x000001b5);
    /// sacute
    pub const sacute: Keysym = Keysym(0x000001b6);
    /// caron
    pub const caron: Keysym = Keysym(0x000001b7);
    /// scaron
    pub const scaron: Keysym = Keysym(0x000001b9);
    /// scedilla
    pub const scedilla: Keysym = Keysym(0x000001ba);
    /// tcaron
    pub const tcaron: Keysym = Keysym(0x000001bb);
    /// zacute
    pub const zacute: Keysym = Keysym(0x000001bc);
    /// doubleacute
    pub const doubleacute: Keysym = Keysym(0x000001bd);
    /// zcaron
    pub const zcaron: Keysym = Keysym(0x000001be);
    /// zabovedot
    pub const zabovedot: Keysym = Keysym(0x000001bf);
    /// Racute
    pub const Racute: Keysym = Keysym(0x000001c0);
    /// Abreve
    pub const Abreve: Keysym = Keysym(0x000001c3);
    /// Lacute
    pub const Lacute: Keysym = Keysym(0x000001c5);
    /// Cacute
    pub const Cacute: Keysym = Keysym(0x000001c6);
    /// Ccaron
    pub const Ccaron: Keysym = Keysym(0x000001c8);
    /// Eogonek
    pub const Eogonek: Keysym = Keysym(0x000001ca);
    /// Ecaron
    pub const Ecaron: Keysym = Keysym(0x000001cc);
    /// Dcaron
    pub const Dcaron: Keysym = Keysym(0x000001cf);
    /// Dstroke
    pub const Dstroke: Keysym = Keysym(0x000001d0);
    /// Nacute
    pub const Nacute: Keysym = Keysym(0x000001d1);
    /// Ncaron
    pub const Ncaron: Keysym = Keysym(0x000001d2);
    /// Odoubleacute
    pub const Odoubleacute: Keysym = Keysym(0x000001d5);
    /// Rcaron
    pub const Rcaron: Keysym = Keysym(0x000001d8);
    /// Uring
    pub const Uring: Keysym = Keysym(0x000001d9);
    /// Udoubleacute
    pub const Udoubleacute: Keysym = Keysym(0x000001db);
    /// Tcedilla
    pub const Tcedilla: Keysym = Keysym(0x000001de);
    /// racute
    pub const racute: Keysym = Keysym(0x000001e0);
    /// abreve
    pub const abreve: Keysym = Keysym(0x000001e3);
    /// lacute
    pub const lacute: Keysym = Keysym(0x000001e5);
    /// cacute
    pub const cacute: Keysym = Keysym(0x000001e6);
    /// ccaron
    pub const ccaron: Keysym = Keysym(0x000001e8);
    /// eogonek
    pub const eogonek: Keysym = Keysym(0x000001ea);
    /// ecaron
    pub const ecaron: Keysym = Keysym(0x000001ec);
    /// dcaron
    pub const dcaron: Keysym = Keysym(0x000001ef);
    /// dstroke
    pub const dstroke: Keysym = Keysym(0x000001f0);
    /// nacute
    pub const nacute: Keysym = Keysym(0x000001f1);
    /// ncaron
    pub const ncaron: Keysym = Keysym(0x000001f2);
    /// odoubleacute
    pub const odoubleacute: Keysym = Keysym(0x000001f5);
    /// rcaron
    pub const rcaron: Keysym = Keysym(0x000001f8);
    /// uring
    pub const uring: Keysym = Keysym(0x000001f9);
    /// udoubleacute
    pub const udoubleacute: Keysym = Keysym(0x000001fb);
    /// tcedilla
    pub const tcedilla: Keysym = Keysym(0x000001fe);
    /// abovedot
    pub const abovedot: Keysym = Keysym(0x000001ff);
    /// Hstroke
    pub const Hstroke: Keysym = Keysym(0x000002a1);
    /// Hcircumflex
    pub const Hcircumflex: Keysym = Keysym(0x000002a6);
    /// Iabovedot
    pub const Iabovedot: Keysym = Keysym(0x000002a9);
    /// Gbreve
    pub const Gbreve: Keysym = Keysym(0x000002ab);
    /// Jcircumflex
    pub const Jcircumflex: Keysym = Keysym(0x000002ac);
    /// hstroke
    pub const hstroke: Keysym = Keysym(0x000002b1);
    /// hcircumflex
    pub const hcircumflex: Keysym = Keysym(0x000002b6);
    /// idotless
    pub const idotless: Keysym = Keysym(0x000002b9);
    /// gbreve
    pub const gbreve: Keysym = Keysym(0x000002bb);
    /// jcircumflex
    pub const jcircumflex: Keysym = Keysym(0x000002bc);
    /// Cabovedot
    pub const Cabovedot: Keysym = Keysym(0x000002c5);
    /// Ccircumflex
    pub const Ccircumflex: Keysym = Keysym(0x000002c6);
    /// Gabovedot
    pub const Gabovedot: Keysym = Keysym(0x000002d5);
    /// Gcircumflex
    pub const Gcircumflex: Keysym = Keysym(0x000002d8);
    /// Ubreve
    pub const Ubreve: Keysym = Keysym(0x000002dd);
    /// Scircumflex
    pub const Scircumflex: Keysym = Keysym(0x000002de);
    /// cabovedot
    pub const cabovedot: Keysym = Keysym(0x000002e5);
    /// ccircumflex
    pub const ccircumflex: Keysym = Keysym(0x000002e6);
    /// gabovedot
    pub const gabovedot: Keysym = Keysym(0x000002f5);
    /// gcircumflex
    pub const gcircumflex: Keysym = Keysym(0x000002f8);
    /// ubreve
    pub const ubreve: Keysym = Keysym(0x000002fd);
    /// scircumflex
    pub const scircumflex: Keysym = Keysym(0x000002fe);
    /// kra
    pub const kra: Keysym = Keysym(0x000003a2);
    /// kappa
    pub const kappa: Keysym = Keysym(0x000003a2);
    /// Rcedilla
    pub const Rcedilla: Keysym = Keysym(0x000003a3);
    /// Itilde
    pub const Itilde: Keysym = Keysym(0x000003a5);
    /// Lcedilla
    pub const Lcedilla: Keysym = Keysym(0x000003a6);
    /// Emacron
    pub const Emacron: Keysym = Keysym(0x000003aa);
    /// Gcedilla
    pub const Gcedilla: Keysym = Keysym(0x000003ab);
    /// Tslash
    pub const Tslash: Keysym = Keysym(0x000003ac);
    /// rcedilla
    pub const rcedilla: Keysym = Keysym(0x000003b3);
    /// itilde
    pub const itilde: Keysym = Keysym(0x000003b5);
    /// lcedilla
    pub const lcedilla: Keysym = Keysym(0x000003b6);
    /// emacron
    pub const emacron: Keysym = Keysym(0x000003ba);
    /// gcedilla
    pub const gcedilla: Keysym = Keysym(0x000003bb);
    /// tslash
    pub const tslash: Keysym = Keysym(0x000003bc);
    /// ENG
    pub const ENG: Keysym = Keysym(0x000003bd);
    /// eng
    pub const eng: Keysym = Keysym(0x000003bf);
    /// Amacron
    pub const Amacron: Keysym = Keysym(0x000003c0);
    /// Iogonek
    pub const Iogonek: Keysym = Keysym(0x000003c7);
    /// Eabovedot
    pub const Eabovedot: Keysym = Keysym(0x000003cc);
    /// Imacron
    pub const Imacron: Keysym = Keysym(0x000003cf);
    /// Ncedilla
    pub const Ncedilla: Keysym = Keysym(0x000003d1);
    /// Omacron
    pub const Omacron: Keysym = Keysym(0x000003d2);
    /// Kcedilla
    pub const Kcedilla: Keysym = Keysym(0x000003d3);
    /// Uogonek
    pub const Uogonek: Keysym = Keysym(0x000003d9);
    /// Utilde
    pub const Utilde: Keysym = Keysym(0x000003dd);
    /// Umacron
    pub const Umacron: Keysym = Keysym(0x000003de);
    /// amacron
    pub const amacron: Keysym = Keysym(0x000003e0);
    /// iogonek
    pub const iogonek: Keysym = Keysym(0x000003e7);
    /// eabovedot
    pub const eabovedot: Keysym = Keysym(0x000003ec);
    /// imacron
    pub const imacron: Keysym = Keysym(0x000003ef);
    /// ncedilla
    pub const ncedilla: Keysym = Keysym(0x000003f1);
    /// omacron
    pub const omacron: Keysym = Keysym(0x000003f2);
    /// kcedilla
    pub const kcedilla: Keysym = Keysym(0x000003f3);
    /// uogonek
    pub const uogonek: Keysym = Keysym(0x000003f9);
    /// utilde
    pub const utilde: Keysym = Keysym(0x000003fd);
    /// umacron
    pub const umacron: Keysym = Keysym(0x000003fe);
    /// overline
    pub const overline: Keysym = Keysym(0x0000047e);
    /// kana_fullstop
    pub const kana_fullstop: Keysym = Keysym(0x000004a1);
    /// kana_openingbracket
    pub const kana_openingbracket: Keysym = Keysym(0x000004a2);
    /// kana_closingbracket
    pub const kana_closingbracket: Keysym = Keysym(0x000004a3);
    /// kana_comma
    pub const kana_comma: Keysym = Keysym(0x000004a4);
    /// kana_conjunctive
    pub const kana_conjunctive: Keysym = Keysym(0x000004a5);
    /// kana_middledot
    pub const kana_middledot: Keysym = Keysym(0x000004a5);
    /// kana_WO
    pub const kana_WO: Keysym = Keysym(0x000004a6);
    /// kana_a
    pub const kana_a: Keysym = Keysym(0x000004a7);
    /// kana_i
    pub const kana_i: Keysym = Keysym(0x000004a8);
    /// kana_u
    pub const kana_u: Keysym = Keysym(0x000004a9);
    /// kana_e
    pub const kana_e: Keysym = Keysym(0x000004aa);
    /// kana_o
    pub const kana_o: Keysym = Keysym(0x000004ab);
    /// kana_ya
    pub const kana_ya: Keysym = Keysym(0x000004ac);
    /// kana_yu
    pub const kana_yu: Keysym = Keysym(0x000004ad);
    /// kana_yo
    pub const kana_yo: Keysym = Keysym(0x000004ae);
    /// kana_tsu
    pub const kana_tsu: Keysym = Keysym(0x000004af);
    /// kana_tu
    pub const kana_tu: Keysym = Keysym(0x000004af);
    /// prolongedsound
    pub const prolongedsound: Keysym = Keysym(0x000004b0);
    /// kana_A
    pub const kana_A: Keysym = Keysym(0x000004b1);
    /// kana_I
    pub const kana_I: Keysym = Keysym(0x000004b2);
    /// kana_U
    pub const kana_U: Keysym = Keysym(0x000004b3);
    /// kana_E
    pub const kana_E: Keysym = Keysym(0x000004b4);
    /// kana_O
    pub const kana_O: Keysym = Keysym(0x000004b5);
    /// kana_KA
    pub const kana_KA: Keysym = Keysym(0x000004b6);
    /// kana_KI
    pub const kana_KI: Keysym = Keysym(0x000004b7);
    /// kana_KU
    pub const kana_KU: Keysym = Keysym(0x000004b8);
    /// kana_KE
    pub const kana_KE: Keysym = Keysym(0x000004b9);
    /// kana_KO
    pub const kana_KO: Keysym = Keysym(0x000004ba);
    /// kana_SA
    pub const kana_SA: Keysym = Keysym(0x000004bb);
    /// kana_SHI
    pub const kana_SHI: Keysym = Keysym(0x000004bc);
    /// kana_SU
    pub const kana_SU: Keysym = Keysym(0x000004bd);
    /// kana_SE
    pub const kana_SE: Keysym = Keysym(0x000004be);
    /// kana_SO
    pub const kana_SO: Keysym = Keysym(0x000004bf);
    /// kana_TA
    pub const kana_TA: Keysym = Keysym(0x000004c0);
    /// kana_CHI
    pub const kana_CHI: Keysym = Keysym(0x000004c1);
    /// kana_TI
    pub const kana_TI: Keysym = Keysym(0x000004c1);
    /// kana_TSU
    pub const kana_TSU: Keysym = Keysym(0x000004c2);
    /// kana_TU
    pub const kana_TU: Keysym = Keysym(0x000004c2);
    /// kana_TE
    pub const kana_TE: Keysym = Keysym(0x000004c3);
    /// kana_TO
    pub const kana_TO: Keysym = Keysym(0x000004c4);
    /// kana_NA
    pub const kana_NA: Keysym = Keysym(0x000004c5);
    /// kana_NI
    pub const kana_NI: Keysym = Keysym(0x000004c6);
    /// kana_NU
    pub const kana_NU: Keysym = Keysym(0x000004c7);
    /// kana_NE
    pub const kana_NE: Keysym = Keysym(0x000004c8);
    /// kana_NO
    pub const kana_NO: Keysym = Keysym(0x000004c9);
    /// kana_HA
    pub const kana_HA: Keysym = Keysym(0x000004ca);
    /// kana_HI
    pub const kana_HI: Keysym = Keysym(0x000004cb);
    /// kana_FU
    pub const kana_FU: Keysym = Keysym(0x000004cc);
    /// kana_HU
    pub const kana_HU: Keysym = Keysym(0x000004cc);
    /// kana_HE
    pub const kana_HE: Keysym = Keysym(0x000004cd);
    /// kana_HO
    pub const kana_HO: Keysym = Keysym(0x000004ce);
    /// kana_MA
    pub const kana_MA: Keysym = Keysym(0x000004cf);
    /// kana_MI
    pub const kana_MI: Keysym = Keysym(0x000004d0);
    /// kana_MU
    pub const kana_MU: Keysym = Keysym(0x000004d1);
    /// kana_ME
    pub const kana_ME: Keysym = Keysym(0x000004d2);
    /// kana_MO
    pub const kana_MO: Keysym = Keysym(0x000004d3);
    /// kana_YA
    pub const kana_YA: Keysym = Keysym(0x000004d4);
    /// kana_YU
    pub const kana_YU: Keysym = Keysym(0x000004d5);
    /// kana_YO
    pub const kana_YO: Keysym = Keysym(0x000004d6);
    /// kana_RA
    pub const kana_RA: Keysym = Keysym(0x000004d7);
    /// kana_RI
    pub const kana_RI: Keysym = Keysym(0x000004d8);
    /// kana_RU
    pub const kana_RU: Keysym = Keysym(0x000004d9);
    /// kana_RE
    pub const kana_RE: Keysym = Keysym(0x000004da);
    /// kana_RO
    pub const kana_RO: Keysym = Keysym(0x000004db);
    /// kana_WA
    pub const kana_WA: Keysym = Keysym(0x000004dc);
    /// kana_N
    pub const kana_N: Keysym = Keysym(0x000004dd);
    /// voicedsound
    pub const voicedsound: Keysym = Keysym(0x000004de);
    /// semivoicedsound
    pub const semivoicedsound: Keysym = Keysym(0x000004df);
    /// Arabic_comma
    pub const Arabic_comma: Keysym = Keysym(0x000005ac);
    /// Arabic_semicolon
    pub const Arabic_semicolon: Keysym = Keysym(0x000005bb);
    /// Arabic_question_mark
    pub const Arabic_question_mark: Keysym = Keysym(0x000005bf);
    /// Arabic_hamza
    pub const Arabic_hamza: Keysym = Keysym(0x000005c1);
    /// Arabic_maddaonalef
    pub const Arabic_maddaonalef: Keysym = Keysym(0x000005c2);
    /// Arabic_hamzaonalef
    pub const Arabic_hamzaonalef: Keysym = Keysym(0x000005c3);
    /// Arabic_hamzaonwaw
    pub const Arabic_hamzaonwaw: Keysym = Keysym(0x000005c4);
    /// Arabic_hamzaunderalef
    pub const Arabic_hamzaunderalef: Keysym = Keysym(0x000005c5);
    /// Arabic_hamzaonyeh
    pub const Arabic_hamzaonyeh: Keysym = Keysym(0x000005c6);
    /// Arabic_alef
    pub const Arabic_alef: Keysym = Keysym(0x000005c7);
    /// Arabic_beh
    pub const Arabic_beh: Keysym = Keysym(0x000005c8);
    /// Arabic_tehmarbuta
    pub const Arabic_tehmarbuta: Keysym = Keysym(0x000005c9);
    /// Arabic_teh
    pub const Arabic_teh: Keysym = Keysym(0x000005ca);
    /// Arabic_theh
    pub const Arabic_theh: Keysym = Keysym(0x000005cb);
    /// Arabic_jeem
    pub const Arabic_jeem: Keysym = Keysym(0x000005cc);
    /// Arabic_hah
    pub const Arabic_hah: Keysym = Keysym(0x000005cd);
    /// Arabic_khah
    pub const Arabic_khah: Keysym = Keysym(0x000005ce);
    /// Arabic_dal
    pub const Arabic_dal: Keysym = Keysym(0x000005cf);
    /// Arabic_thal
    pub const Arabic_thal: Keysym = Keysym(0x000005d0);
    /// Arabic_ra
    pub const Arabic_ra: Keysym = Keysym(0x000005d1);
    /// Arabic_zain
    pub const Arabic_zain: Keysym = Keysym(0x000005d2);
    /// Arabic_seen
    pub const Arabic_seen: Keysym = Keysym(0x000005d3);
    /// Arabic_sheen
    pub const Arabic_sheen: Keysym = Keysym(0x000005d4);
    /// Arabic_sad
    pub const Arabic_sad: Keysym = Keysym(0x000005d5);
    /// Arabic_dad
    pub const Arabic_dad: Keysym = Keysym(0x000005d6);
    /// Arabic_tah
    pub const Arabic_tah: Keysym = Keysym(0x000005d7);
    /// Arabic_zah
    pub const Arabic_zah: Keysym = Keysym(0x000005d8);
    /// Arabic_ain
    pub const Arabic_ain: Keysym = Keysym(0x000005d9);
    /// Arabic_ghain
    pub const Arabic_ghain: Keysym = Keysym(0x000005da);
    /// Arabic_tatweel
    pub const Arabic_tatweel: Keysym = Keysym(0x000005e0);
    /// Arabic_feh
    pub const Arabic_feh: Keysym = Keysym(0x000005e1);
    /// Arabic_qaf
    pub const Arabic_qaf: Keysym = Keysym(0x000005e2);
    /// Arabic_kaf
    pub const Arabic_kaf: Keysym = Keysym(0x000005e3);
    /// Arabic_lam
    pub const Arabic_lam: Keysym = Keysym(0x000005e4);
    /// Arabic_meem
    pub const Arabic_meem: Keysym = Keysym(0x000005e5);
    /// Arabic_noon
    pub const Arabic_noon: Keysym = Keysym(0x000005e6);
    /// Arabic_ha
    pub const Arabic_ha: Keysym = Keysym(0x000005e7);
    /// Arabic_heh
    pub const Arabic_heh: Keysym = Keysym(0x000005e7);
    /// Arabic_waw
    pub const Arabic_waw: Keysym = Keysym(0x000005e8);
    /// Arabic_alefmaksura
    pub const Arabic_alefmaksura: Keysym = Keysym(0x000005e9);
    /// Arabic_yeh
    pub const Arabic_yeh: Keysym = Keysym(0x000005ea);
    /// Arabic_fathatan
    pub const Arabic_fathatan: Keysym = Keysym(0x000005eb);
    /// Arabic_dammatan
    pub const Arabic_dammatan: Keysym = Keysym(0x000005ec);
    /// Arabic_kasratan
    pub const Arabic_kasratan: Keysym = Keysym(0x000005ed);
    /// Arabic_fatha
    pub const Arabic_fatha: Keysym = Keysym(0x000005ee);
    /// Arabic_damma
    pub const Arabic_damma: Keysym = Keysym(0x000005ef);
    /// Arabic_kasra
    pub const Arabic_kasra: Keysym = Keysym(0x000005f0);
    /// Arabic_shadda
    pub const Arabic_shadda: Keysym = Keysym(0x000005f1);
    /// Arabic_sukun
    pub const Arabic_sukun: Keysym = Keysym(0x000005f2);
    /// Serbian_dje
    pub const Serbian_dje: Keysym = Keysym(0x000006a1);
    /// Macedonia_gje
    pub const Macedonia_gje: Keysym = Keysym(0x000006a2);
    /// Cyrillic_io
    pub const Cyrillic_io: Keysym = Keysym(0x000006a3);
    /// Ukrainian_ie
    pub const Ukrainian_ie: Keysym = Keysym(0x000006a4);
    /// Ukranian_je
    pub const Ukranian_je: Keysym = Keysym(0x000006a4);
    /// Macedonia_dse
    pub const Macedonia_dse: Keysym = Keysym(0x000006a5);
    /// Ukrainian_i
    pub const Ukrainian_i: Keysym = Keysym(0x000006a6);
    /// Ukranian_i
    pub const Ukranian_i: Keysym = Keysym(0x000006a6);
    /// Ukrainian_yi
    pub const Ukrainian_yi: Keysym = Keysym(0x000006a7);
    /// Ukranian_yi
    pub const Ukranian_yi: Keysym = Keysym(0x000006a7);
    /// Cyrillic_je
    pub const Cyrillic_je: Keysym = Keysym(0x000006a8);
    /// Serbian_je
    pub const Serbian_je: Keysym = Keysym(0x000006a8);
    /// Cyrillic_lje
    pub const Cyrillic_lje: Keysym = Keysym(0x000006a9);
    /// Serbian_lje
    pub const Serbian_lje: Keysym = Keysym(0x000006a9);
    /// Cyrillic_nje
    pub const Cyrillic_nje: Keysym = Keysym(0x000006aa);
    /// Serbian_nje
    pub const Serbian_nje: Keysym = Keysym(0x000006aa);
    /// Serbian_tshe
    pub const Serbian_tshe: Keysym = Keysym(0x000006ab);
    /// Macedonia_kje
    pub const Macedonia_kje: Keysym = Keysym(0x000006ac);
    /// Ukrainian_ghe_with_upturn
    pub const Ukrainian_ghe_with_upturn: Keysym = Keysym(0x000006ad);
    /// Byelorussian_shortu
    pub const Byelorussian_shortu: Keysym = Keysym(0x000006ae);
    /// Cyrillic_dzhe
    pub const Cyrillic_dzhe: Keysym = Keysym(0x000006af);
    /// Serbian_dze
    pub const Serbian_dze: Keysym = Keysym(0x000006af);
    /// numerosign
    pub const numerosign: Keysym = Keysym(0x000006b0);
    /// Serbian_DJE
    pub const Serbian_DJE: Keysym = Keysym(0x000006b1);
    /// Macedonia_GJE
    pub const Macedonia_GJE: Keysym = Keysym(0x000006b2);
    /// Cyrillic_IO
    pub const Cyrillic_IO: Keysym = Keysym(0x000006b3);
    /// Ukrainian_IE
    pub const Ukrainian_IE: Keysym = Keysym(0x000006b4);
    /// Ukranian_JE
    pub const Ukranian_JE: Keysym = Keysym(0x000006b4);
    /// Macedonia_DSE
    pub const Macedonia_DSE: Keysym = Keysym(0x000006b5);
    /// Ukrainian_I
    pub const Ukrainian_I: Keysym = Keysym(0x000006b6);
    /// Ukranian_I
    pub const Ukranian_I: Keysym = Keysym(0x000006b6);
    /// Ukrainian_YI
    pub const Ukrainian_YI: Keysym = Keysym(0x000006b7);
    /// Ukranian_YI
    pub const Ukranian_YI: Keysym = Keysym(0x000006b7);
    /// Cyrillic_JE
    pub const Cyrillic_JE: Keysym = Keysym(0x000006b8);
    /// Serbian_JE
    pub const Serbian_JE: Keysym = Keysym(0x000006b8);
    /// Cyrillic_LJE
    pub const Cyrillic_LJE: Keysym = Keysym(0x000006b9);
    /// Serbian_LJE
    pub const Serbian_LJE: Keysym = Keysym(0x000006b9);
    /// Cyrillic_NJE
    pub const Cyrillic_NJE: Keysym = Keysym(0x000006ba);
    /// Serbian_NJE
    pub const Serbian_NJE: Keysym = Keysym(0x000006ba);
    /// Serbian_TSHE
    pub const Serbian_TSHE: Keysym = Keysym(0x000006bb);
    /// Macedonia_KJE
    pub const Macedonia_KJE: Keysym = Keysym(0x000006bc);
    /// Ukrainian_GHE_WITH_UPTURN
    pub const Ukrainian_GHE_WITH_UPTURN: Keysym = Keysym(0x000006bd);
    /// Byelorussian_SHORTU
    pub const Byelorussian_SHORTU: Keysym = Keysym(0x000006be);
    /// Cyrillic_DZHE
    pub const Cyrillic_DZHE: Keysym = Keysym(0x000006bf);
    /// Serbian_DZE
    pub const Serbian_DZE: Keysym = Keysym(0x000006bf);
    /// Cyrillic_yu
    pub const Cyrillic_yu: Keysym = Keysym(0x000006c0);
    /// Cyrillic_a
    pub const Cyrillic_a: Keysym = Keysym(0x000006c1);
    /// Cyrillic_be
    pub const Cyrillic_be: Keysym = Keysym(0x000006c2);
    /// Cyrillic_tse
    pub const Cyrillic_tse: Keysym = Keysym(0x000006c3);
    /// Cyrillic_de
    pub const Cyrillic_de: Keysym = Keysym(0x000006c4);
    /// Cyrillic_ie
    pub const Cyrillic_ie: Keysym = Keysym(0x000006c5);
    /// Cyrillic_ef
    pub const Cyrillic_ef: Keysym = Keysym(0x000006c6);
    /// Cyrillic_ghe
    pub const Cyrillic_ghe: Keysym = Keysym(0x000006c7);
    /// Cyrillic_ha
    pub const Cyrillic_ha: Keysym = Keysym(0x000006c8);
    /// Cyrillic_i
    pub const Cyrillic_i: Keysym = Keysym(0x000006c9);
    /// Cyrillic_shorti
    pub const Cyrillic_shorti: Keysym = Keysym(0x000006ca);
    /// Cyrillic_ka
    pub const Cyrillic_ka: Keysym = Keysym(0x000006cb);
    /// Cyrillic_el
    pub const Cyrillic_el: Keysym = Keysym(0x000006cc);
    /// Cyrillic_em
    pub const Cyrillic_em: Keysym = Keysym(0x000006cd);
    /// Cyrillic_en
    pub const Cyrillic_en: Keysym = Keysym(0x000006ce);
    /// Cyrillic_o
    pub const Cyrillic_o: Keysym = Keysym(0x000006cf);
    /// Cyrillic_pe
    pub const Cyrillic_pe: Keysym = Keysym(0x000006d0);
    /// Cyrillic_ya
    pub const Cyrillic_ya: Keysym = Keysym(0x000006d1);
    /// Cyrillic_er
    pub const Cyrillic_er: Keysym = Keysym(0x000006d2);
    /// Cyrillic_es
    pub const Cyrillic_es: Keysym = Keysym(0x000006d3);
    /// Cyrillic_te
    pub const Cyrillic_te: Keysym = Keysym(0x000006d4);
    /// Cyrillic_u
    pub const Cyrillic_u: Keysym = Keysym(0x000006d5);
    /// Cyrillic_zhe
    pub const Cyrillic_zhe: Keysym = Keysym(0x000006d6);
    /// Cyrillic_ve
    pub const Cyrillic_ve: Keysym = Keysym(0x000006d7);
    /// Cyrillic_softsign
    pub const Cyrillic_softsign: Keysym = Keysym(0x000006d8);
    /// Cyrillic_yeru
    pub const Cyrillic_yeru: Keysym = Keysym(0x000006d9);
    /// Cyrillic_ze
    pub const Cyrillic_ze: Keysym = Keysym(0x000006da);
    /// Cyrillic_sha
    pub const Cyrillic_sha: Keysym = Keysym(0x000006db);
    /// Cyrillic_e
    pub const Cyrillic_e: Keysym = Keysym(0x000006dc);
    /// Cyrillic_shcha
    pub const Cyrillic_shcha: Keysym = Keysym(0x000006dd);
    /// Cyrillic_che
    pub const Cyrillic_che: Keysym = Keysym(0x000006de);
    /// Cyrillic_hardsign
    pub const Cyrillic_hardsign: Keysym = Keysym(0x000006df);
    /// Cyrillic_YU
    pub const Cyrillic_YU: Keysym = Keysym(0x000006e0);
    /// Cyrillic_A
    pub const Cyrillic_A: Keysym = Keysym(0x000006e1);
    /// Cyrillic_BE
    pub const Cyrillic_BE: Keysym = Keysym(0x000006e2);
    /// Cyrillic_TSE
    pub const Cyrillic_TSE: Keysym = Keysym(0x000006e3);
    /// Cyrillic_DE
    pub const Cyrillic_DE: Keysym = Keysym(0x000006e4);
    /// Cyrillic_IE
    pub const Cyrillic_IE: Keysym = Keysym(0x000006e5);
    /// Cyrillic_EF
    pub const Cyrillic_EF: Keysym = Keysym(0x000006e6);
    /// Cyrillic_GHE
    pub const Cyrillic_GHE: Keysym = Keysym(0x000006e7);
    /// Cyrillic_HA
    pub const Cyrillic_HA: Keysym = Keysym(0x000006e8);
    /// Cyrillic_I
    pub const Cyrillic_I: Keysym = Keysym(0x000006e9);
    /// Cyrillic_SHORTI
    pub const Cyrillic_SHORTI: Keysym = Keysym(0x000006ea);
    /// Cyrillic_KA
    pub const Cyrillic_KA: Keysym = Keysym(0x000006eb);
    /// Cyrillic_EL
    pub const Cyrillic_EL: Keysym = Keysym(0x000006ec);
    /// Cyrillic_EM
    pub const Cyrillic_EM: Keysym = Keysym(0x000006ed);
    /// Cyrillic_EN
    pub const Cyrillic_EN: Keysym = Keysym(0x000006ee);
    /// Cyrillic_O
    pub const Cyrillic_O: Keysym = Keysym(0x000006ef);
    /// Cyrillic_PE
    pub const Cyrillic_PE: Keysym = Keysym(0x000006f0);
    /// Cyrillic_YA
    pub const Cyrillic_YA: Keysym = Keysym(0x000006f1);
    /// Cyrillic_ER
    pub const Cyrillic_ER: Keysym = Keysym(0x000006f2);
    /// Cyrillic_ES
    pub const Cyrillic_ES: Keysym = Keysym(0x000006f3);
    /// Cyrillic_TE
    pub const Cyrillic_TE: Keysym = Keysym(0x000006f4);
    /// Cyrillic_U
    pub const Cyrillic_U: Keysym = Keysym(0x000006f5);
    /// Cyrillic_ZHE
    pub const Cyrillic_ZHE: Keysym = Keysym(0x000006f6);
    /// Cyrillic_VE
    pub const Cyrillic_VE: Keysym = Keysym(0x000006f7);
    /// Cyrillic_SOFTSIGN
    pub const Cyrillic_SOFTSIGN: Keysym = Keysym(0x000006f8);
    /// Cyrillic_YERU
    pub const Cyrillic_YERU: Keysym = Keysym(0x000006f9);
    /// Cyrillic_ZE
    pub const Cyrillic_ZE: Keysym = Keysym(0x000006fa);
    /// Cyrillic_SHA
    pub const Cyrillic_SHA: Keysym = Keysym(0x000006fb);
    /// Cyrillic_E
    pub const Cyrillic_E: Keysym = Keysym(0x000006fc);
    /// Cyrillic_SHCHA
    pub const Cyrillic_SHCHA: Keysym = Keysym(0x000006fd);
    /// Cyrillic_CHE
    pub const Cyrillic_CHE: Keysym = Keysym(0x000006fe);
    /// Cyrillic_HARDSIGN
    pub const Cyrillic_HARDSIGN: Keysym = Keysym(0x000006ff);
    /// Greek_ALPHAaccent
    pub const Greek_ALPHAaccent: Keysym = Keysym(0x000007a1);
    /// Greek_EPSILONaccent
    pub const Greek_EPSILONaccent: Keysym = Keysym(0x000007a2);
    /// Greek_ETAaccent
    pub const Greek_ETAaccent: Keysym = Keysym(0x000007a3);
    /// Greek_IOTAaccent
    pub const Greek_IOTAaccent: Keysym = Keysym(0x000007a4);
    /// Greek_IOTAdieresis
    pub const Greek_IOTAdieresis: Keysym = Keysym(0x000007a5);
    /// Greek_IOTAdiaeresis
    pub const Greek_IOTAdiaeresis: Keysym = Keysym(0x000007a5);
    /// Greek_OMICRONaccent
    pub const Greek_OMICRONaccent: Keysym = Keysym(0x000007a7);
    /// Greek_UPSILONaccent
    pub const Greek_UPSILONaccent: Keysym = Keysym(0x000007a8);
    /// Greek_UPSILONdieresis
    pub const Greek_UPSILONdieresis: Keysym = Keysym(0x000007a9);
    /// Greek_OMEGAaccent
    pub const Greek_OMEGAaccent: Keysym = Keysym(0x000007ab);
    /// Greek_accentdieresis
    pub const Greek_accentdieresis: Keysym = Keysym(0x000007ae);
    /// Greek_horizbar
    pub const Greek_horizbar: Keysym = Keysym(0x000007af);
    /// Greek_alphaaccent
    pub const Greek_alphaaccent: Keysym = Keysym(0x000007b1);
    /// Greek_epsilonaccent
    pub const Greek_epsilonaccent: Keysym = Keysym(0x000007b2);
    /// Greek_etaaccent
    pub const Greek_etaaccent: Keysym = Keysym(0x000007b3);
    /// Greek_iotaaccent
    pub const Greek_iotaaccent: Keysym = Keysym(0x000007b4);
    /// Greek_iotadieresis
    pub const Greek_iotadieresis: Keysym = Keysym(0x000007b5);
    /// Greek_iotaaccentdieresis
    pub const Greek_iotaaccentdieresis: Keysym = Keysym(0x000007b6);
    /// Greek_omicronaccent
    pub const Greek_omicronaccent: Keysym = Keysym(0x000007b7);
    /// Greek_upsilonaccent
    pub const Greek_upsilonaccent: Keysym = Keysym(0x000007b8);
    /// Greek_upsilondieresis
    pub const Greek_upsilondieresis: Keysym = Keysym(0x000007b9);
    /// Greek_upsilonaccentdieresis
    pub const Greek_upsilonaccentdieresis: Keysym = Keysym(0x000007ba);
    /// Greek_omegaaccent
    pub const Greek_omegaaccent: Keysym = Keysym(0x000007bb);
    /// Greek_ALPHA
    pub const Greek_ALPHA: Keysym = Keysym(0x000007c1);
    /// Greek_BETA
    pub const Greek_BETA: Keysym = Keysym(0x000007c2);
    /// Greek_GAMMA
    pub const Greek_GAMMA: Keysym = Keysym(0x000007c3);
    /// Greek_DELTA
    pub const Greek_DELTA: Keysym = Keysym(0x000007c4);
    /// Greek_EPSILON
    pub const Greek_EPSILON: Keysym = Keysym(0x000007c5);
    /// Greek_ZETA
    pub const Greek_ZETA: Keysym = Keysym(0x000007c6);
    /// Greek_ETA
    pub const Greek_ETA: Keysym = Keysym(0x000007c7);
    /// Greek_THETA
    pub const Greek_THETA: Keysym = Keysym(0x000007c8);
    /// Greek_IOTA
    pub const Greek_IOTA: Keysym = Keysym(0x000007c9);
    /// Greek_KAPPA
    pub const Greek_KAPPA: Keysym = Keysym(0x000007ca);
    /// Greek_LAMDA
    pub const Greek_LAMDA: Keysym = Keysym(0x000007cb);
    /// Greek_LAMBDA
    pub const Greek_LAMBDA: Keysym = Keysym(0x000007cb);
    /// Greek_MU
    pub const Greek_MU: Keysym = Keysym(0x000007cc);
    /// Greek_NU
    pub const Greek_NU: Keysym = Keysym(0x000007cd);
    /// Greek_XI
    pub const Greek_XI: Keysym = Keysym(0x000007ce);
    /// Greek_OMICRON
    pub const Greek_OMICRON: Keysym = Keysym(0x000007cf);
    /// Greek_PI
    pub const Greek_PI: Keysym = Keysym(0x000007d0);
    /// Greek_RHO
    pub const Greek_RHO: Keysym = Keysym(0x000007d1);
    /// Greek_SIGMA
    pub const Greek_SIGMA: Keysym = Keysym(0x000007d2);
    /// Greek_TAU
    pub const Greek_TAU: Keysym = Keysym(0x000007d4);
    /// Greek_UPSILON
    pub const Greek_UPSILON: Keysym = Keysym(0x000007d5);
    /// Greek_PHI
    pub const Greek_PHI: Keysym = Keysym(0x000007d6);
    /// Greek_CHI
    pub const Greek_CHI: Keysym = Keysym(0x000007d7);
    /// Greek_PSI
    pub const Greek_PSI: Keysym = Keysym(0x000007d8);
    /// Greek_OMEGA
    pub const Greek_OMEGA: Keysym = Keysym(0x000007d9);
    /// Greek_alpha
    pub const Greek_alpha: Keysym = Keysym(0x000007e1);
    /// Greek_beta
    pub const Greek_beta: Keysym = Keysym(0x000007e2);
    /// Greek_gamma
    pub const Greek_gamma: Keysym = Keysym(0x000007e3);
    /// Greek_delta
    pub const Greek_delta: Keysym = Keysym(0x000007e4);
    /// Greek_epsilon
    pub const Greek_epsilon: Keysym = Keysym(0x000007e5);
    /// Greek_zeta
    pub const Greek_zeta: Keysym = Keysym(0x000007e6);
    /// Greek_eta
    pub const Greek_eta: Keysym = Keysym(0x000007e7);
    /// Greek_theta
    pub const Greek_theta: Keysym = Keysym(0x000007e8);
    /// Greek_iota
    pub const Greek_iota: Keysym = Keysym(0x000007e9);
    /// Greek_kappa
    pub const Greek_kappa: Keysym = Keysym(0x000007ea);
    /// Greek_lamda
    pub const Greek_lamda: Keysym = Keysym(0x000007eb);
    /// Greek_lambda
    pub const Greek_lambda: Keysym = Keysym(0x000007eb);
    /// Greek_mu
    pub const Greek_mu: Keysym = Keysym(0x000007ec);
    /// Greek_nu
    pub const Greek_nu: Keysym = Keysym(0x000007ed);
    /// Greek_xi
    pub const Greek_xi: Keysym = Keysym(0x000007ee);
    /// Greek_omicron
    pub const Greek_omicron: Keysym = Keysym(0x000007ef);
    /// Greek_pi
    pub const Greek_pi: Keysym = Keysym(0x000007f0);
    /// Greek_rho
    pub const Greek_rho: Keysym = Keysym(0x000007f1);
    /// Greek_sigma
    pub const Greek_sigma: Keysym = Keysym(0x000007f2);
    /// Greek_finalsmallsigma
    pub const Greek_finalsmallsigma: Keysym = Keysym(0x000007f3);
    /// Greek_tau
    pub const Greek_tau: Keysym = Keysym(0x000007f4);
    /// Greek_upsilon
    pub const Greek_upsilon: Keysym = Keysym(0x000007f5);
    /// Greek_phi
    pub const Greek_phi: Keysym = Keysym(0x000007f6);
    /// Greek_chi
    pub const Greek_chi: Keysym = Keysym(0x000007f7);
    /// Greek_psi
    pub const Greek_psi: Keysym = Keysym(0x000007f8);
    /// Greek_omega
    pub const Greek_omega: Keysym = Keysym(0x000007f9);
    /// leftradical
    pub const leftradical: Keysym = Keysym(0x000008a1);
    /// topleftradical
    pub const topleftradical: Keysym = Keysym(0x000008a2);
    /// horizconnector
    pub const horizconnector: Keysym = Keysym(0x000008a3);
    /// topintegral
    pub const topintegral: Keysym = Keysym(0x000008a4);
    /// botintegral
    pub const botintegral: Keysym = Keysym(0x000008a5);
    /// vertconnector
    pub const vertconnector: Keysym = Keysym(0x000008a6);
    /// topleftsqbracket
    pub const topleftsqbracket: Keysym = Keysym(0x000008a7);
    /// botleftsqbracket
    pub const botleftsqbracket: Keysym = Keysym(0x000008a8);
    /// toprightsqbracket
    pub const toprightsqbracket: Keysym = Keysym(0x000008a9);
    /// botrightsqbracket
    pub const botrightsqbracket: Keysym = Keysym(0x000008aa);
    /// topleftparens
    pub const topleftparens: Keysym = Keysym(0x000008ab);
    /// botleftparens
    pub const botleftparens: Keysym = Keysym(0x000008ac);
    /// toprightparens
    pub const toprightparens: Keysym = Keysym(0x000008ad);
    /// botrightparens
    pub const botrightparens: Keysym = Keysym(0x000008ae);
    /// leftmiddlecurlybrace
    pub const leftmiddlecurlybrace: Keysym = Keysym(0x000008af);
    /// rightmiddlecurlybrace
    pub const rightmiddlecurlybrace: Keysym = Keysym(0x000008b0);
    /// topleftsummation
    pub const topleftsummation: Keysym = Keysym(0x000008b1);
    /// botleftsummation
    pub const botleftsummation: Keysym = Keysym(0x000008b2);
    /// topvertsummationconnector
    pub const topvertsummationconnector: Keysym = Keysym(0x000008b3);
    /// botvertsummationconnector
    pub const botvertsummationconnector: Keysym = Keysym(0x000008b4);
    /// toprightsummation
    pub const toprightsummation: Keysym = Keysym(0x000008b5);
    /// botrightsummation
    pub const botrightsummation: Keysym = Keysym(0x000008b6);
    /// rightmiddlesummation
    pub const rightmiddlesummation: Keysym = Keysym(0x000008b7);
    /// lessthanequal
    pub const lessthanequal: Keysym = Keysym(0x000008bc);
    /// notequal
    pub const notequal: Keysym = Keysym(0x000008bd);
    /// greaterthanequal
    pub const greaterthanequal: Keysym = Keysym(0x000008be);
    /// integral
    pub const integral: Keysym = Keysym(0x000008bf);
    /// therefore
    pub const therefore: Keysym = Keysym(0x000008c0);
    /// variation
    pub const variation: Keysym = Keysym(0x000008c1);
    /// infinity
    pub const infinity: Keysym = Keysym(0x000008c2);
    /// nabla
    pub const nabla: Keysym = Keysym(0x000008c5);
    /// approximate
    pub const approximate: Keysym = Keysym(0x000008c8);
    /// similarequal
    pub const similarequal: Keysym = Keysym(0x000008c9);
    /// ifonlyif
    pub const ifonlyif: Keysym = Keysym(0x000008cd);
    /// implies
    pub const implies: Keysym = Keysym(0x000008ce);
    /// identical
    pub const identical: Keysym = Keysym(0x000008cf);
    /// radical
    pub const radical: Keysym = Keysym(0x000008d6);
    /// includedin
    pub const includedin: Keysym = Keysym(0x000008da);
    /// includes
    pub const includes: Keysym = Keysym(0x000008db);
    /// intersection
    pub const intersection: Keysym = Keysym(0x000008dc);
    /// union
    pub const union: Keysym = Keysym(0x000008dd);
    /// logicaland
    pub const logicaland: Keysym = Keysym(0x000008de);
    /// logicalor
    pub const logicalor: Keysym = Keysym(0x000008df);
    /// partialderivative
    pub const partialderivative: Keysym = Keysym(0x000008ef);
    /// function
    pub const function: Keysym = Keysym(0x000008f6);
    /// leftarrow
    pub const leftarrow: Keysym = Keysym(0x000008fb);
    /// uparrow
    pub const uparrow: Keysym = Keysym(0x000008fc);
    /// rightarrow
    pub const rightarrow: Keysym = Keysym(0x000008fd);
    /// downarrow
    pub const downarrow: Keysym = Keysym(0x000008fe);
    /// blank
    pub const blank: Keysym = Keysym(0x000009df);
    /// soliddiamond
    pub const soliddiamond: Keysym = Keysym(0x000009e0);
    /// checkerboard
    pub const checkerboard: Keysym = Keysym(0x000009e1);
    /// ht
    pub const ht: Keysym = Keysym(0x000009e2);
    /// ff
    pub const ff: Keysym = Keysym(0x000009e3);
    /// cr
    pub const cr: Keysym = Keysym(0x000009e4);
    /// lf
    pub const lf: Keysym = Keysym(0x000009e5);
    /// nl
    pub const nl: Keysym = Keysym(0x000009e8);
    /// vt
    pub const vt: Keysym = Keysym(0x000009e9);
    /// lowrightcorner
    pub const lowrightcorner: Keysym = Keysym(0x000009ea);
    /// uprightcorner
    pub const uprightcorner: Keysym = Keysym(0x000009eb);
    /// upleftcorner
    pub const upleftcorner: Keysym = Keysym(0x000009ec);
    /// lowleftcorner
    pub const lowleftcorner: Keysym = Keysym(0x000009ed);
    /// crossinglines
    pub const crossinglines: Keysym = Keysym(0x000009ee);
    /// horizlinescan1
    pub const horizlinescan1: Keysym = Keysym(0x000009ef);
    /// horizlinescan3
    pub const horizlinescan3: Keysym = Keysym(0x000009f0);
    /// horizlinescan5
    pub const horizlinescan5: Keysym = Keysym(0x000009f1);
    /// horizlinescan7
    pub const horizlinescan7: Keysym = Keysym(0x000009f2);
    /// horizlinescan9
    pub const horizlinescan9: Keysym = Keysym(0x000009f3);
    /// leftt
    pub const leftt: Keysym = Keysym(0x000009f4);
    /// rightt
    pub const rightt: Keysym = Keysym(0x000009f5);
    /// bott
    pub const bott: Keysym = Keysym(0x000009f6);
    /// topt
    pub const topt: Keysym = Keysym(0x000009f7);
    /// vertbar
    pub const vertbar: Keysym = Keysym(0x000009f8);
    /// emspace
    pub const emspace: Keysym = Keysym(0x00000aa1);
    /// enspace
    pub const enspace: Keysym = Keysym(0x00000aa2);
    /// em3space
    pub const em3space: Keysym = Keysym(0x00000aa3);
    /// em4space
    pub const em4space: Keysym = Keysym(0x00000aa4);
    /// digitspace
    pub const digitspace: Keysym = Keysym(0x00000aa5);
    /// punctspace
    pub const punctspace: Keysym = Keysym(0x00000aa6);
    /// thinspace
    pub const thinspace: Keysym = Keysym(0x00000aa7);
    /// hairspace
    pub const hairspace: Keysym = Keysym(0x00000aa8);
    /// emdash
    pub const emdash: Keysym = Keysym(0x00000aa9);
    /// endash
    pub const endash: Keysym = Keysym(0x00000aaa);
    /// signifblank
    pub const signifblank: Keysym = Keysym(0x00000aac);
    /// ellipsis
    pub const ellipsis: Keysym = Keysym(0x00000aae);
    /// doubbaselinedot
    pub const doubbaselinedot: Keysym = Keysym(0x00000aaf);
    /// onethird
    pub const onethird: Keysym = Keysym(0x00000ab0);
    /// twothirds
    pub const twothirds: Keysym = Keysym(0x00000ab1);
    /// onefifth
    pub const onefifth: Keysym = Keysym(0x00000ab2);
    /// twofifths
    pub const twofifths: Keysym = Keysym(0x00000ab3);
    /// threefifths
    pub const threefifths: Keysym = Keysym(0x00000ab4);
    /// fourfifths
    pub const fourfifths: Keysym = Keysym(0x00000ab5);
    /// onesixth
    pub const onesixth: Keysym = Keysym(0x00000ab6);
    /// fivesixths
    pub const fivesixths: Keysym = Keysym(0x00000ab7);
    /// careof
    pub const careof: Keysym = Keysym(0x00000ab8);
    /// figdash
    pub const figdash: Keysym = Keysym(0x00000abb);
    /// leftanglebracket
    pub const leftanglebracket: Keysym = Keysym(0x00000abc);
    /// decimalpoint
    pub const decimalpoint: Keysym = Keysym(0x00000abd);
    /// rightanglebracket
    pub const rightanglebracket: Keysym = Keysym(0x00000abe);
    /// marker
    pub const marker: Keysym = Keysym(0x00000abf);
    /// oneeighth
    pub const oneeighth: Keysym = Keysym(0x00000ac3);
    /// threeeighths
    pub const threeeighths: Keysym = Keysym(0x00000ac4);
    /// fiveeighths
    pub const fiveeighths: Keysym = Keysym(0x00000ac5);
    /// seveneighths
    pub const seveneighths: Keysym = Keysym(0x00000ac6);
    /// trademark
    pub const trademark: Keysym = Keysym(0x00000ac9);
    /// signaturemark
    pub const signaturemark: Keysym = Keysym(0x00000aca);
    /// trademarkincircle
    pub const trademarkincircle: Keysym = Keysym(0x00000acb);
    /// leftopentriangle
    pub const leftopentriangle: Keysym = Keysym(0x00000acc);
    /// rightopentriangle
    pub const rightopentriangle: Keysym = Keysym(0x00000acd);
    /// emopencircle
    pub const emopencircle: Keysym = Keysym(0x00000ace);
    /// emopenrectangle
    pub const emopenrectangle: Keysym = Keysym(0x00000acf);
    /// leftsinglequotemark
    pub const leftsinglequotemark: Keysym = Keysym(0x00000ad0);
    /// rightsinglequotemark
    pub const rightsinglequotemark: Keysym = Keysym(0x00000ad1);
    /// leftdoublequotemark
    pub const leftdoublequotemark: Keysym = Keysym(0x00000ad2);
    /// rightdoublequotemark
    pub const rightdoublequotemark: Keysym = Keysym(0x00000ad3);
    /// prescription
    pub const prescription: Keysym = Keysym(0x00000ad4);
    /// permille
    pub const permille: Keysym = Keysym(0x00000ad5);
    /// minutes
    pub const minutes: Keysym = Keysym(0x00000ad6);
    /// seconds
    pub const seconds: Keysym = Keysym(0x00000ad7);
    /// latincross
    pub const latincross: Keysym = Keysym(0x00000ad9);
    /// hexagram
    pub const hexagram: Keysym = Keysym(0x00000ada);
    /// filledrectbullet
    pub const filledrectbullet: Keysym = Keysym(0x00000adb);
    /// filledlefttribullet
    pub const filledlefttribullet: Keysym = Keysym(0x00000adc);
    /// filledrighttribullet
    pub const filledrighttribullet: Keysym = Keysym(0x00000add);
    /// emfilledcircle
    pub const emfilledcircle: Keysym = Keysym(0x00000ade);
    /// emfilledrect
    pub const emfilledrect: Keysym = Keysym(0x00000adf);
    /// enopencircbullet
    pub const enopencircbullet: Keysym = Keysym(0x00000ae0);
    /// enopensquarebullet
    pub const enopensquarebullet: Keysym = Keysym(0x00000ae1);
    /// openrectbullet
    pub const openrectbullet: Keysym = Keysym(0x00000ae2);
    /// opentribulletup
    pub const opentribulletup: Keysym = Keysym(0x00000ae3);
    /// opentribulletdown
    pub const opentribulletdown: Keysym = Keysym(0x00000ae4);
    /// openstar
    pub const openstar: Keysym = Keysym(0x00000ae5);
    /// enfilledcircbullet
    pub const enfilledcircbullet: Keysym = Keysym(0x00000ae6);
    /// enfilledsqbullet
    pub const enfilledsqbullet: Keysym = Keysym(0x00000ae7);
    /// filledtribulletup
    pub const filledtribulletup: Keysym = Keysym(0x00000ae8);
    /// filledtribulletdown
    pub const filledtribulletdown: Keysym = Keysym(0x00000ae9);
    /// leftpointer
    pub const leftpointer: Keysym = Keysym(0x00000aea);
    /// rightpointer
    pub const rightpointer: Keysym = Keysym(0x00000aeb);
    /// club
    pub const club: Keysym = Keysym(0x00000aec);
    /// diamond
    pub const diamond: Keysym = Keysym(0x00000aed);
    /// heart
    pub const heart: Keysym = Keysym(0x00000aee);
    /// maltesecross
    pub const maltesecross: Keysym = Keysym(0x00000af0);
    /// dagger
    pub const dagger: Keysym = Keysym(0x00000af1);
    /// doubledagger
    pub const doubledagger: Keysym = Keysym(0x00000af2);
    /// checkmark
    pub const checkmark: Keysym = Keysym(0x00000af3);
    /// ballotcross
    pub const ballotcross: Keysym = Keysym(0x00000af4);
    /// musicalsharp
    pub const musicalsharp: Keysym = Keysym(0x00000af5);
    /// musicalflat
    pub const musicalflat: Keysym = Keysym(0x00000af6);
    /// malesymbol
    pub const malesymbol: Keysym = Keysym(0x00000af7);
    /// femalesymbol
    pub const femalesymbol: Keysym = Keysym(0x00000af8);
    /// telephone
    pub const telephone: Keysym = Keysym(0x00000af9);
    /// telephonerecorder
    pub const telephonerecorder: Keysym = Keysym(0x00000afa);
    /// phonographcopyright
    pub const phonographcopyright: Keysym = Keysym(0x00000afb);
    /// caret
    pub const caret: Keysym = Keysym(0x00000afc);
    /// singlelowquotemark
    pub const singlelowquotemark: Keysym = Keysym(0x00000afd);
    /// doublelowquotemark
    pub const doublelowquotemark: Keysym = Keysym(0x00000afe);
    /// cursor
    pub const cursor: Keysym = Keysym(0x00000aff);
    /// leftcaret
    pub const leftcaret: Keysym = Keysym(0x00000ba3);
    /// rightcaret
    pub const rightcaret: Keysym = Keysym(0x00000ba6);
    /// downcaret
    pub const downcaret: Keysym = Keysym(0x00000ba8);
    /// upcaret
    pub const upcaret: Keysym = Keysym(0x00000ba9);
    /// overbar
    pub const overbar: Keysym = Keysym(0x00000bc0);
    /// downtack
    pub const downtack: Keysym = Keysym(0x00000bc2);
    /// upshoe
    pub const upshoe: Keysym = Keysym(0x00000bc3);
    /// downstile
    pub const downstile: Keysym = Keysym(0x00000bc4);
    /// underbar
    pub const underbar: Keysym = Keysym(0x00000bc6);
    /// jot
    pub const jot: Keysym = Keysym(0x00000bca);
    /// quad
    pub const quad: Keysym = Keysym(0x00000bcc);
    /// uptack
    pub const uptack: Keysym = Keysym(0x00000bce);
    /// circle
    pub const circle: Keysym = Keysym(0x00000bcf);
    /// upstile
    pub const upstile: Keysym = Keysym(0x00000bd3);
    /// downshoe
    pub const downshoe: Keysym = Keysym(0x00000bd6);
    /// rightshoe
    pub const rightshoe: Keysym = Keysym(0x00000bd8);
    /// leftshoe
    pub const leftshoe: Keysym = Keysym(0x00000bda);
    /// lefttack
    pub const lefttack: Keysym = Keysym(0x00000bdc);
    /// righttack
    pub const righttack: Keysym = Keysym(0x00000bfc);
    /// hebrew_doublelowline
    pub const hebrew_doublelowline: Keysym = Keysym(0x00000cdf);
    /// hebrew_aleph
    pub const hebrew_aleph: Keysym = Keysym(0x00000ce0);
    /// hebrew_bet
    pub const hebrew_bet: Keysym = Keysym(0x00000ce1);
    /// hebrew_beth
    pub const hebrew_beth: Keysym = Keysym(0x00000ce1);
    /// hebrew_gimel
    pub const hebrew_gimel: Keysym = Keysym(0x00000ce2);
    /// hebrew_gimmel
    pub const hebrew_gimmel: Keysym = Keysym(0x00000ce2);
    /// hebrew_dalet
    pub const hebrew_dalet: Keysym = Keysym(0x00000ce3);
    /// hebrew_daleth
    pub const hebrew_daleth: Keysym = Keysym(0x00000ce3);
    /// hebrew_he
    pub const hebrew_he: Keysym = Keysym(0x00000ce4);
    /// hebrew_waw
    pub const hebrew_waw: Keysym = Keysym(0x00000ce5);
    /// hebrew_zain
    pub const hebrew_zain: Keysym = Keysym(0x00000ce6);
    /// hebrew_zayin
    pub const hebrew_zayin: Keysym = Keysym(0x00000ce6);
    /// hebrew_chet
    pub const hebrew_chet: Keysym = Keysym(0x00000ce7);
    /// hebrew_het
    pub const hebrew_het: Keysym = Keysym(0x00000ce7);
    /// hebrew_tet
    pub const hebrew_tet: Keysym = Keysym(0x00000ce8);
    /// hebrew_teth
    pub const hebrew_teth: Keysym = Keysym(0x00000ce8);
    /// hebrew_yod
    pub const hebrew_yod: Keysym = Keysym(0x00000ce9);
    /// hebrew_finalkaph
    pub const hebrew_finalkaph: Keysym = Keysym(0x00000cea);
    /// hebrew_kaph
    pub const hebrew_kaph: Keysym = Keysym(0x00000ceb);
    /// hebrew_lamed
    pub const hebrew_lamed: Keysym = Keysym(0x00000cec);
    /// hebrew_finalmem
    pub const hebrew_finalmem: Keysym = Keysym(0x00000ced);
    /// hebrew_mem
    pub const hebrew_mem: Keysym = Keysym(0x00000cee);
    /// hebrew_finalnun
    pub const hebrew_finalnun: Keysym = Keysym(0x00000cef);
    /// hebrew_nun
    pub const hebrew_nun: Keysym = Keysym(0x00000cf0);
    /// hebrew_samech
    pub const hebrew_samech: Keysym = Keysym(0x00000cf1);
    /// hebrew_samekh
    pub const hebrew_samekh: Keysym = Keysym(0x00000cf1);
    /// hebrew_ayin
    pub const hebrew_ayin: Keysym = Keysym(0x00000cf2);
    /// hebrew_finalpe
    pub const hebrew_finalpe: Keysym = Keysym(0x00000cf3);
    /// hebrew_pe
    pub const hebrew_pe: Keysym = Keysym(0x00000cf4);
    /// hebrew_finalzade
    pub const hebrew_finalzade: Keysym = Keysym(0x00000cf5);
    /// hebrew_finalzadi
    pub const hebrew_finalzadi: Keysym = Keysym(0x00000cf5);
    /// hebrew_zade
    pub const hebrew_zade: Keysym = Keysym(0x00000cf6);
    /// hebrew_zadi
    pub const hebrew_zadi: Keysym = Keysym(0x00000cf6);
    /// hebrew_qoph
    pub const hebrew_qoph: Keysym = Keysym(0x00000cf7);
    /// hebrew_kuf
    pub const hebrew_kuf: Keysym = Keysym(0x00000cf7);
    /// hebrew_resh
    pub const hebrew_resh: Keysym = Keysym(0x00000cf8);
    /// hebrew_shin
    pub const hebrew_shin: Keysym = Keysym(0x00000cf9);
    /// hebrew_taw
    pub const hebrew_taw: Keysym = Keysym(0x00000cfa);
    /// hebrew_taf
    pub const hebrew_taf: Keysym = Keysym(0x00000cfa);
    /// Thai_kokai
    pub const Thai_kokai: Keysym = Keysym(0x00000da1);
    /// Thai_khokhai
    pub const Thai_khokhai: Keysym = Keysym(0x00000da2);
    /// Thai_khokhuat
    pub const Thai_khokhuat: Keysym = Keysym(0x00000da3);
    /// Thai_khokhwai
    pub const Thai_khokhwai: Keysym = Keysym(0x00000da4);
    /// Thai_khokhon
    pub const Thai_khokhon: Keysym = Keysym(0x00000da5);
    /// Thai_khorakhang
    pub const Thai_khorakhang: Keysym = Keysym(0x00000da6);
    /// Thai_ngongu
    pub const Thai_ngongu: Keysym = Keysym(0x00000da7);
    /// Thai_chochan
    pub const Thai_chochan: Keysym = Keysym(0x00000da8);
    /// Thai_choching
    pub const Thai_choching: Keysym = Keysym(0x00000da9);
    /// Thai_chochang
    pub const Thai_chochang: Keysym = Keysym(0x00000daa);
    /// Thai_soso
    pub const Thai_soso: Keysym = Keysym(0x00000dab);
    /// Thai_chochoe
    pub const Thai_chochoe: Keysym = Keysym(0x00000dac);
    /// Thai_yoying
    pub const Thai_yoying: Keysym = Keysym(0x00000dad);
    /// Thai_dochada
    pub const Thai_dochada: Keysym = Keysym(0x00000dae);
    /// Thai_topatak
    pub const Thai_topatak: Keysym = Keysym(0x00000daf);
    /// Thai_thothan
    pub const Thai_thothan: Keysym = Keysym(0x00000db0);
    /// Thai_thonangmontho
    pub const Thai_thonangmontho: Keysym = Keysym(0x00000db1);
    /// Thai_thophuthao
    pub const Thai_thophuthao: Keysym = Keysym(0x00000db2);
    /// Thai_nonen
    pub const Thai_nonen: Keysym = Keysym(0x00000db3);
    /// Thai_dodek
    pub const Thai_dodek: Keysym = Keysym(0x00000db4);
    /// Thai_totao
    pub const Thai_totao: Keysym = Keysym(0x00000db5);
    /// Thai_thothung
    pub const Thai_thothung: Keysym = Keysym(0x00000db6);
    /// Thai_thothahan
    pub const Thai_thothahan: Keysym = Keysym(0x00000db7);
    /// Thai_thothong
    pub const Thai_thothong: Keysym = Keysym(0x00000db8);
    /// Thai_nonu
    pub const Thai_nonu: Keysym = Keysym(0x00000db9);
    /// Thai_bobaimai
    pub const Thai_bobaimai: Keysym = Keysym(0x00000dba);
    /// Thai_popla
    pub const Thai_popla: Keysym = Keysym(0x00000dbb);
    /// Thai_phophung
    pub const Thai_phophung: Keysym = Keysym(0x00000dbc);
    /// Thai_fofa
    pub const Thai_fofa: Keysym = Keysym(0x00000dbd);
    /// Thai_phophan
    pub const Thai_phophan: Keysym = Keysym(0x00000dbe);
    /// Thai_fofan
    pub const Thai_fofan: Keysym = Keysym(0x00000dbf);
    /// Thai_phosamphao
    pub const Thai_phosamphao: Keysym = Keysym(0x00000dc0);
    /// Thai_moma
    pub const Thai_moma: Keysym = Keysym(0x00000dc1);
    /// Thai_yoyak
    pub const Thai_yoyak: Keysym = Keysym(0x00000dc2);
    /// Thai_rorua
    pub const Thai_rorua: Keysym = Keysym(0x00000dc3);
    /// Thai_ru
    pub const Thai_ru: Keysym = Keysym(0x00000dc4);
    /// Thai_loling
    pub const Thai_loling: Keysym = Keysym(0x00000dc5);
    /// Thai_lu
    pub const Thai_lu: Keysym = Keysym(0x00000dc6);
    /// Thai_wowaen
    pub const Thai_wowaen: Keysym = Keysym(0x00000dc7);
    /// Thai_sosala
    pub const Thai_sosala: Keysym = Keysym(0x00000dc8);
    /// Thai_sorusi
    pub const Thai_sorusi: Keysym = Keysym(0x00000dc9);
    /// Thai_sosua
    pub const Thai_sosua: Keysym = Keysym(0x00000dca);
    /// Thai_hohip
    pub const Thai_hohip: Keysym = Keysym(0x00000dcb);
    /// Thai_lochula
    pub const Thai_lochula: Keysym = Keysym(0x00000dcc);
    /// Thai_oang
    pub const Thai_oang: Keysym = Keysym(0x00000dcd);
    /// Thai_honokhuk
    pub const Thai_honokhuk: Keysym = Keysym(0x00000dce);
    /// Thai_paiyannoi
    pub const Thai_paiyannoi: Keysym = Keysym(0x00000dcf);
    /// Thai_saraa
    pub const Thai_saraa: Keysym = Keysym(0x00000dd0);
    /// Thai_maihanakat
    pub const Thai_maihanakat: Keysym = Keysym(0x00000dd1);
    /// Thai_saraaa
    pub const Thai_saraaa: Keysym = Keysym(0x00000dd2);
    /// Thai_saraam
    pub const Thai_saraam: Keysym = Keysym(0x00000dd3);
    /// Thai_sarai
    pub const Thai_sarai: Keysym = Keysym(0x00000dd4);
    /// Thai_saraii
    pub const Thai_saraii: Keysym = Keysym(0x00000dd5);
    /// Thai_saraue
    pub const Thai_saraue: Keysym = Keysym(0x00000dd6);
    /// Thai_sarauee
    pub const Thai_sarauee: Keysym = Keysym(0x00000dd7);
    /// Thai_sarau
    pub const Thai_sarau: Keysym = Keysym(0x00000dd8);
    /// Thai_sarauu
    pub const Thai_sarauu: Keysym = Keysym(0x00000dd9);
    /// Thai_phinthu
    pub const Thai_phinthu: Keysym = Keysym(0x00000dda);
    /// Thai_maihanakat_maitho
    pub const Thai_maihanakat_maitho: Keysym = Keysym(0x00000dde);
    /// Thai_baht
    pub const Thai_baht: Keysym = Keysym(0x00000ddf);
    /// Thai_sarae
    pub const Thai_sarae: Keysym = Keysym(0x00000de0);
    /// Thai_saraae
    pub const Thai_saraae: Keysym = Keysym(0x00000de1);
    /// Thai_sarao
    pub const Thai_sarao: Keysym = Keysym(0x00000de2);
    /// Thai_saraaimaimuan
    pub const Thai_saraaimaimuan: Keysym = Keysym(0x00000de3);
    /// Thai_saraaimaimalai
    pub const Thai_saraaimaimalai: Keysym = Keysym(0x00000de4);
    /// Thai_lakkhangyao
    pub const Thai_lakkhangyao: Keysym = Keysym(0x00000de5);
    /// Thai_maiyamok
    pub const Thai_maiyamok: Keysym = Keysym(0x00000de6);
    /// Thai_maitaikhu
    pub const Thai_maitaikhu: Keysym = Keysym(0x00000de7);
    /// Thai_maiek
    pub const Thai_maiek: Keysym = Keysym(0x00000de8);
    /// Thai_maitho
    pub const Thai_maitho: Keysym = Keysym(0x00000de9);
    /// Thai_maitri
    pub const Thai_maitri: Keysym = Keysym(0x00000dea);
    /// Thai_maichattawa
    pub const Thai_maichattawa: Keysym = Keysym(0x00000deb);
    /// Thai_thanthakhat
    pub const Thai_thanthakhat: Keysym = Keysym(0x00000dec);
    /// Thai_nikhahit
    pub const Thai_nikhahit: Keysym = Keysym(0x00000ded);
    /// Thai_leksun
    pub const Thai_leksun: Keysym = Keysym(0x00000df0);
    /// Thai_leknung
    pub const Thai_leknung: Keysym = Keysym(0x00000df1);
    /// Thai_leksong
    pub const Thai_leksong: Keysym = Keysym(0x00000df2);
    /// Thai_leksam
    pub const Thai_leksam: Keysym = Keysym(0x00000df3);
    /// Thai_leksi
    pub const Thai_leksi: Keysym = Keysym(0x00000df4);
    /// Thai_lekha
    pub const Thai_lekha: Keysym = Keysym(0x00000df5);
    /// Thai_lekhok
    pub const Thai_lekhok: Keysym = Keysym(0x00000df6);
    /// Thai_lekchet
    pub const Thai_lekchet: Keysym = Keysym(0x00000df7);
    /// Thai_lekpaet
    pub const Thai_lekpaet: Keysym = Keysym(0x00000df8);
    /// Thai_lekkao
    pub const Thai_lekkao: Keysym = Keysym(0x00000df9);
    /// Hangul_Kiyeog
    pub const Hangul_Kiyeog: Keysym = Keysym(0x00000ea1);
    /// Hangul_SsangKiyeog
    pub const Hangul_SsangKiyeog: Keysym = Keysym(0x00000ea2);
    /// Hangul_KiyeogSios
    pub const Hangul_KiyeogSios: Keysym = Keysym(0x00000ea3);
    /// Hangul_Nieun
    pub const Hangul_Nieun: Keysym = Keysym(0x00000ea4);
    /// Hangul_NieunJieuj
    pub const Hangul_NieunJieuj: Keysym = Keysym(0x00000ea5);
    /// Hangul_NieunHieuh
    pub const Hangul_NieunHieuh: Keysym = Keysym(0x00000ea6);
    /// Hangul_Dikeud
    pub const Hangul_Dikeud: Keysym = Keysym(0x00000ea7);
    /// Hangul_SsangDikeud
    pub const Hangul_SsangDikeud: Keysym = Keysym(0x00000ea8);
    /// Hangul_Rieul
    pub const Hangul_Rieul: Keysym = Keysym(0x00000ea9);
    /// Hangul_RieulKiyeog
    pub const Hangul_RieulKiyeog: Keysym = Keysym(0x00000eaa);
    /// Hangul_RieulMieum
    pub const Hangul_RieulMieum: Keysym = Keysym(0x00000eab);
    /// Hangul_RieulPieub
    pub const Hangul_RieulPieub: Keysym = Keysym(0x00000eac);
    /// Hangul_RieulSios
    pub const Hangul_RieulSios: Keysym = Keysym(0x00000ead);
    /// Hangul_RieulTieut
    pub const Hangul_RieulTieut: Keysym = Keysym(0x00000eae);
    /// Hangul_RieulPhieuf
    pub const Hangul_RieulPhieuf: Keysym = Keysym(0x00000eaf);
    /// Hangul_RieulHieuh
    pub const Hangul_RieulHieuh: Keysym = Keysym(0x00000eb0);
    /// Hangul_Mieum
    pub const Hangul_Mieum: Keysym = Keysym(0x00000eb1);
    /// Hangul_Pieub
    pub const Hangul_Pieub: Keysym = Keysym(0x00000eb2);
    /// Hangul_SsangPieub
    pub const Hangul_SsangPieub: Keysym = Keysym(0x00000eb3);
    /// Hangul_PieubSios
    pub const Hangul_PieubSios: Keysym = Keysym(0x00000eb4);
    /// Hangul_Sios
    pub const Hangul_Sios: Keysym = Keysym(0x00000eb5);
    /// Hangul_SsangSios
    pub const Hangul_SsangSios: Keysym = Keysym(0x00000eb6);
    /// Hangul_Ieung
    pub const Hangul_Ieung: Keysym = Keysym(0x00000eb7);
    /// Hangul_Jieuj
    pub const Hangul_Jieuj: Keysym = Keysym(0x00000eb8);
    /// Hangul_SsangJieuj
    pub const Hangul_SsangJieuj: Keysym = Keysym(0x00000eb9);
    /// Hangul_Cieuc
    pub const Hangul_Cieuc: Keysym = Keysym(0x00000eba);
    /// Hangul_Khieuq
    pub const Hangul_Khieuq: Keysym = Keysym(0x00000ebb);
    /// Hangul_Tieut
    pub const Hangul_Tieut: Keysym = Keysym(0x00000ebc);
    /// Hangul_Phieuf
    pub const Hangul_Phieuf: Keysym = Keysym(0x00000ebd);
    /// Hangul_Hieuh
    pub const Hangul_Hieuh: Keysym = Keysym(0x00000ebe);
    /// Hangul_A
    pub const Hangul_A: Keysym = Keysym(0x00000ebf);
    /// Hangul_AE
    pub const Hangul_AE: Keysym = Keysym(0x00000ec0);
    /// Hangul_YA
    pub const Hangul_YA: Keysym = Keysym(0x00000ec1);
    /// Hangul_YAE
    pub const Hangul_YAE: Keysym = Keysym(0x00000ec2);
    /// Hangul_EO
    pub const Hangul_EO: Keysym = Keysym(0x00000ec3);
    /// Hangul_E
    pub const Hangul_E: Keysym = Keysym(0x00000ec4);
    /// Hangul_YEO
    pub const Hangul_YEO: Keysym = Keysym(0x00000ec5);
    /// Hangul_YE
    pub const Hangul_YE: Keysym = Keysym(0x00000ec6);
    /// Hangul_O
    pub const Hangul_O: Keysym = Keysym(0x00000ec7);
    /// Hangul_WA
    pub const Hangul_WA: Keysym = Keysym(0x00000ec8);
    /// Hangul_WAE
    pub const Hangul_WAE: Keysym = Keysym(0x00000ec9);
    /// Hangul_OE
    pub const Hangul_OE: Keysym = Keysym(0x00000eca);
    /// Hangul_YO
    pub const Hangul_YO: Keysym = Keysym(0x00000ecb);
    /// Hangul_U
    pub const Hangul_U: Keysym = Keysym(0x00000ecc);
    /// Hangul_WEO
    pub const Hangul_WEO: Keysym = Keysym(0x00000ecd);
    /// Hangul_WE
    pub const Hangul_WE: Keysym = Keysym(0x00000ece);
    /// Hangul_WI
    pub const Hangul_WI: Keysym = Keysym(0x00000ecf);
    /// Hangul_YU
    pub const Hangul_YU: Keysym = Keysym(0x00000ed0);
    /// Hangul_EU
    pub const Hangul_EU: Keysym = Keysym(0x00000ed1);
    /// Hangul_YI
    pub const Hangul_YI: Keysym = Keysym(0x00000ed2);
    /// Hangul_I
    pub const Hangul_I: Keysym = Keysym(0x00000ed3);
    /// Hangul_J_Kiyeog
    pub const Hangul_J_Kiyeog: Keysym = Keysym(0x00000ed4);
    /// Hangul_J_SsangKiyeog
    pub const Hangul_J_SsangKiyeog: Keysym = Keysym(0x00000ed5);
    /// Hangul_J_KiyeogSios
    pub const Hangul_J_KiyeogSios: Keysym = Keysym(0x00000ed6);
    /// Hangul_J_Nieun
    pub const Hangul_J_Nieun: Keysym = Keysym(0x00000ed7);
    /// Hangul_J_NieunJieuj
    pub const Hangul_J_NieunJieuj: Keysym = Keysym(0x00000ed8);
    /// Hangul_J_NieunHieuh
    pub const Hangul_J_NieunHieuh: Keysym = Keysym(0x00000ed9);
    /// Hangul_J_Dikeud
    pub const Hangul_J_Dikeud: Keysym = Keysym(0x00000eda);
    /// Hangul_J_Rieul
    pub const Hangul_J_Rieul: Keysym = Keysym(0x00000edb);
    /// Hangul_J_RieulKiyeog
    pub const Hangul_J_RieulKiyeog: Keysym = Keysym(0x00000edc);
    /// Hangul_J_RieulMieum
    pub const Hangul_J_RieulMieum: Keysym = Keysym(0x00000edd);
    /// Hangul_J_RieulPieub
    pub const Hangul_J_RieulPieub: Keysym = Keysym(0x00000ede);
    /// Hangul_J_RieulSios
    pub const Hangul_J_RieulSios: Keysym = Keysym(0x00000edf);
    /// Hangul_J_RieulTieut
    pub const Hangul_J_RieulTieut: Keysym = Keysym(0x00000ee0);
    /// Hangul_J_RieulPhieuf
    pub const Hangul_J_RieulPhieuf: Keysym = Keysym(0x00000ee1);
    /// Hangul_J_RieulHieuh
    pub const Hangul_J_RieulHieuh: Keysym = Keysym(0x00000ee2);
    /// Hangul_J_Mieum
    pub const Hangul_J_Mieum: Keysym = Keysym(0x00000ee3);
    /// Hangul_J_Pieub
    pub const Hangul_J_Pieub: Keysym = Keysym(0x00000ee4);
    /// Hangul_J_PieubSios
    pub const Hangul_J_PieubSios: Keysym = Keysym(0x00000ee5);
    /// Hangul_J_Sios
    pub const Hangul_J_Sios: Keysym = Keysym(0x00000ee6);
    /// Hangul_J_SsangSios
    pub const Hangul_J_SsangSios: Keysym = Keysym(0x00000ee7);
    /// Hangul_J_Ieung
    pub const Hangul_J_Ieung: Keysym = Keysym(0x00000ee8);
    /// Hangul_J_Jieuj
    pub const Hangul_J_Jieuj: Keysym = Keysym(0x00000ee9);
    /// Hangul_J_Cieuc
    pub const Hangul_J_Cieuc: Keysym = Keysym(0x00000eea);
    /// Hangul_J_Khieuq
    pub const Hangul_J_Khieuq: Keysym = Keysym(0x00000eeb);
    /// Hangul_J_Tieut
    pub const Hangul_J_Tieut: Keysym = Keysym(0x00000eec);
    /// Hangul_J_Phieuf
    pub const Hangul_J_Phieuf: Keysym = Keysym(0x00000eed);
    /// Hangul_J_Hieuh
    pub const Hangul_J_Hieuh: Keysym = Keysym(0x00000eee);
    /// Hangul_RieulYeorinHieuh
    pub const Hangul_RieulYeorinHieuh: Keysym = Keysym(0x00000eef);
    /// Hangul_SunkyeongeumMieum
    pub const Hangul_SunkyeongeumMieum: Keysym = Keysym(0x00000ef0);
    /// Hangul_SunkyeongeumPieub
    pub const Hangul_SunkyeongeumPieub: Keysym = Keysym(0x00000ef1);
    /// Hangul_PanSios
    pub const Hangul_PanSios: Keysym = Keysym(0x00000ef2);
    /// Hangul_KkogjiDalrinIeung
    pub const Hangul_KkogjiDalrinIeung: Keysym = Keysym(0x00000ef3);
    /// Hangul_SunkyeongeumPhieuf
    pub const Hangul_SunkyeongeumPhieuf: Keysym = Keysym(0x00000ef4);
    /// Hangul_YeorinHieuh
    pub const Hangul_YeorinHieuh: Keysym = Keysym(0x00000ef5);
    /// Hangul_AraeA
    pub const Hangul_AraeA: Keysym = Keysym(0x00000ef6);
    /// Hangul_AraeAE
    pub const Hangul_AraeAE: Keysym = Keysym(0x00000ef7);
    /// Hangul_J_PanSios
    pub const Hangul_J_PanSios: Keysym = Keysym(0x00000ef8);
    /// Hangul_J_KkogjiDalrinIeung
    pub const Hangul_J_KkogjiDalrinIeung: Keysym = Keysym(0x00000ef9);
    /// Hangul_J_YeorinHieuh
    pub const Hangul_J_YeorinHieuh: Keysym = Keysym(0x00000efa);
    /// Korean_Won
    pub const Korean_Won: Keysym = Keysym(0x00000eff);
    /// OE
    pub const OE: Keysym = Keysym(0x000013bc);
    /// oe
    pub const oe: Keysym = Keysym(0x000013bd);
    /// Ydiaeresis
    pub const Ydiaeresis: Keysym = Keysym(0x000013be);
    /// EuroSign
    pub const EuroSign: Keysym = Keysym(0x000020ac);
    /// 3270_Duplicate
    pub const _3270_Duplicate: Keysym = Keysym(0x0000fd01);
    /// 3270_FieldMark
    pub const _3270_FieldMark: Keysym = Keysym(0x0000fd02);
    /// 3270_Right2
    pub const _3270_Right2: Keysym = Keysym(0x0000fd03);
    /// 3270_Left2
    pub const _3270_Left2: Keysym = Keysym(0x0000fd04);
    /// 3270_BackTab
    pub const _3270_BackTab: Keysym = Keysym(0x0000fd05);
    /// 3270_EraseEOF
    pub const _3270_EraseEOF: Keysym = Keysym(0x0000fd06);
    /// 3270_EraseInput
    pub const _3270_EraseInput: Keysym = Keysym(0x0000fd07);
    /// 3270_Reset
    pub const _3270_Reset: Keysym = Keysym(0x0000fd08);
    /// 3270_Quit
    pub const _3270_Quit: Keysym = Keysym(0x0000fd09);
    /// 3270_PA1
    pub const _3270_PA1: Keysym = Keysym(0x0000fd0a);
    /// 3270_PA2
    pub const _3270_PA2: Keysym = Keysym(0x0000fd0b);
    /// 3270_PA3
    pub const _3270_PA3: Keysym = Keysym(0x0000fd0c);
    /// 3270_Test
    pub const _3270_Test: Keysym = Keysym(0x0000fd0d);
    /// 3270_Attn
    pub const _3270_Attn: Keysym = Keysym(0x0000fd0e);
    /// 3270_CursorBlink
    pub const _3270_CursorBlink: Keysym = Keysym(0x0000fd0f);
    /// 3270_AltCursor
    pub const _3270_AltCursor: Keysym = Keysym(0x0000fd10);
    /// 3270_KeyClick
    pub const _3270_KeyClick: Keysym = Keysym(0x0000fd11);
    /// 3270_Jump
    pub const _3270_Jump: Keysym = Keysym(0x0000fd12);
    /// 3270_Ident
    pub const _3270_Ident: Keysym = Keysym(0x0000fd13);
    /// 3270_Rule
    pub const _3270_Rule: Keysym = Keysym(0x0000fd14);
    /// 3270_Copy
    pub const _3270_Copy: Keysym = Keysym(0x0000fd15);
    /// 3270_Play
    pub const _3270_Play: Keysym = Keysym(0x0000fd16);
    /// 3270_Setup
    pub const _3270_Setup: Keysym = Keysym(0x0000fd17);
    /// 3270_Record
    pub const _3270_Record: Keysym = Keysym(0x0000fd18);
    /// 3270_ChangeScreen
    pub const _3270_ChangeScreen: Keysym = Keysym(0x0000fd19);
    /// 3270_DeleteWord
    pub const _3270_DeleteWord: Keysym = Keysym(0x0000fd1a);
    /// 3270_ExSelect
    pub const _3270_ExSelect: Keysym = Keysym(0x0000fd1b);
    /// 3270_CursorSelect
    pub const _3270_CursorSelect: Keysym = Keysym(0x0000fd1c);
    /// 3270_PrintScreen
    pub const _3270_PrintScreen: Keysym = Keysym(0x0000fd1d);
    /// 3270_Enter
    pub const _3270_Enter: Keysym = Keysym(0x0000fd1e);
    /// ISO_Lock
    pub const ISO_Lock: Keysym = Keysym(0x0000fe01);
    /// ISO_Level2_Latch
    pub const ISO_Level2_Latch: Keysym = Keysym(0x0000fe02);
    /// ISO_Level3_Shift
    pub const ISO_Level3_Shift: Keysym = Keysym(0x0000fe03);
    /// ISO_Level3_Latch
    pub const ISO_Level3_Latch: Keysym = Keysym(0x0000fe04);
    /// ISO_Level3_Lock
    pub const ISO_Level3_Lock: Keysym = Keysym(0x0000fe05);
    /// ISO_Group_Latch
    pub const ISO_Group_Latch: Keysym = Keysym(0x0000fe06);
    /// ISO_Group_Lock
    pub const ISO_Group_Lock: Keysym = Keysym(0x0000fe07);
    /// ISO_Next_Group
    pub const ISO_Next_Group: Keysym = Keysym(0x0000fe08);
    /// ISO_Next_Group_Lock
    pub const ISO_Next_Group_Lock: Keysym = Keysym(0x0000fe09);
    /// ISO_Prev_Group
    pub const ISO_Prev_Group: Keysym = Keysym(0x0000fe0a);
    /// ISO_Prev_Group_Lock
    pub const ISO_Prev_Group_Lock: Keysym = Keysym(0x0000fe0b);
    /// ISO_First_Group
    pub const ISO_First_Group: Keysym = Keysym(0x0000fe0c);
    /// ISO_First_Group_Lock
    pub const ISO_First_Group_Lock: Keysym = Keysym(0x0000fe0d);
    /// ISO_Last_Group
    pub const ISO_Last_Group: Keysym = Keysym(0x0000fe0e);
    /// ISO_Last_Group_Lock
    pub const ISO_Last_Group_Lock: Keysym = Keysym(0x0000fe0f);
    /// ISO_Level5_Shift
    pub const ISO_Level5_Shift: Keysym = Keysym(0x0000fe11);
    /// ISO_Level5_Latch
    pub const ISO_Level5_Latch: Keysym = Keysym(0x0000fe12);
    /// ISO_Level5_Lock
    pub const ISO_Level5_Lock: Keysym = Keysym(0x0000fe13);
    /// ISO_Left_Tab
    pub const ISO_Left_Tab: Keysym = Keysym(0x0000fe20);
    /// ISO_Move_Line_Up
    pub const ISO_Move_Line_Up: Keysym = Keysym(0x0000fe21);
    /// ISO_Move_Line_Down
    pub const ISO_Move_Line_Down: Keysym = Keysym(0x0000fe22);
    /// ISO_Partial_Line_Up
    pub const ISO_Partial_Line_Up: Keysym = Keysym(0x0000fe23);
    /// ISO_Partial_Line_Down
    pub const ISO_Partial_Line_Down: Keysym = Keysym(0x0000fe24);
    /// ISO_Partial_Space_Left
    pub const ISO_Partial_Space_Left: Keysym = Keysym(0x0000fe25);
    /// ISO_Partial_Space_Right
    pub const ISO_Partial_Space_Right: Keysym = Keysym(0x0000fe26);
    /// ISO_Set_Margin_Left
    pub const ISO_Set_Margin_Left: Keysym = Keysym(0x0000fe27);
    /// ISO_Set_Margin_Right
    pub const ISO_Set_Margin_Right: Keysym = Keysym(0x0000fe28);
    /// ISO_Release_Margin_Left
    pub const ISO_Release_Margin_Left: Keysym = Keysym(0x0000fe29);
    /// ISO_Release_Margin_Right
    pub const ISO_Release_Margin_Right: Keysym = Keysym(0x0000fe2a);
    /// ISO_Release_Both_Margins
    pub const ISO_Release_Both_Margins: Keysym = Keysym(0x0000fe2b);
    /// ISO_Fast_Cursor_Left
    pub const ISO_Fast_Cursor_Left: Keysym = Keysym(0x0000fe2c);
    /// ISO_Fast_Cursor_Right
    pub const ISO_Fast_Cursor_Right: Keysym = Keysym(0x0000fe2d);
    /// ISO_Fast_Cursor_Up
    pub const ISO_Fast_Cursor_Up: Keysym = Keysym(0x0000fe2e);
    /// ISO_Fast_Cursor_Down
    pub const ISO_Fast_Cursor_Down: Keysym = Keysym(0x0000fe2f);
    /// ISO_Continuous_Underline
    pub const ISO_Continuous_Underline: Keysym = Keysym(0x0000fe30);
    /// ISO_Discontinuous_Underline
    pub const ISO_Discontinuous_Underline: Keysym = Keysym(0x0000fe31);
    /// ISO_Emphasize
    pub const ISO_Emphasize: Keysym = Keysym(0x0000fe32);
    /// ISO_Center_Object
    pub const ISO_Center_Object: Keysym = Keysym(0x0000fe33);
    /// ISO_Enter
    pub const ISO_Enter: Keysym = Keysym(0x0000fe34);
    /// dead_grave
    pub const dead_grave: Keysym = Keysym(0x0000fe50);
    /// dead_acute
    pub const dead_acute: Keysym = Keysym(0x0000fe51);
    /// dead_circumflex
    pub const dead_circumflex: Keysym = Keysym(0x0000fe52);
    /// dead_tilde
    pub const dead_tilde: Keysym = Keysym(0x0000fe53);
    /// dead_perispomeni
    pub const dead_perispomeni: Keysym = Keysym(0x0000fe53);
    /// dead_macron
    pub const dead_macron: Keysym = Keysym(0x0000fe54);
    /// dead_breve
    pub const dead_breve: Keysym = Keysym(0x0000fe55);
    /// dead_abovedot
    pub const dead_abovedot: Keysym = Keysym(0x0000fe56);
    /// dead_diaeresis
    pub const dead_diaeresis: Keysym = Keysym(0x0000fe57);
    /// dead_abovering
    pub const dead_abovering: Keysym = Keysym(0x0000fe58);
    /// dead_doubleacute
    pub const dead_doubleacute: Keysym = Keysym(0x0000fe59);
    /// dead_caron
    pub const dead_caron: Keysym = Keysym(0x0000fe5a);
    /// dead_cedilla
    pub const dead_cedilla: Keysym = Keysym(0x0000fe5b);
    /// dead_ogonek
    pub const dead_ogonek: Keysym = Keysym(0x0000fe5c);
    /// dead_iota
    pub const dead_iota: Keysym = Keysym(0x0000fe5d);
    /// dead_voiced_sound
    pub const dead_voiced_sound: Keysym = Keysym(0x0000fe5e);
    /// dead_semivoiced_sound
    pub const dead_semivoiced_sound: Keysym = Keysym(0x0000fe5f);
    /// dead_belowdot
    pub const dead_belowdot: Keysym = Keysym(0x0000fe60);
    /// dead_hook
    pub const dead_hook: Keysym = Keysym(0x0000fe61);
    /// dead_horn
    pub const dead_horn: Keysym = Keysym(0x0000fe62);
    /// dead_stroke
    pub const dead_stroke: Keysym = Keysym(0x0000fe63);
    /// dead_abovecomma
    pub const dead_abovecomma: Keysym = Keysym(0x0000fe64);
    /// dead_psili
    pub const dead_psili: Keysym = Keysym(0x0000fe64);
    /// dead_abovereversedcomma
    pub const dead_abovereversedcomma: Keysym = Keysym(0x0000fe65);
    /// dead_dasia
    pub const dead_dasia: Keysym = Keysym(0x0000fe65);
    /// dead_doublegrave
    pub const dead_doublegrave: Keysym = Keysym(0x0000fe66);
    /// dead_belowring
    pub const dead_belowring: Keysym = Keysym(0x0000fe67);
    /// dead_belowmacron
    pub const dead_belowmacron: Keysym = Keysym(0x0000fe68);
    /// dead_belowcircumflex
    pub const dead_belowcircumflex: Keysym = Keysym(0x0000fe69);
    /// dead_belowtilde
    pub const dead_belowtilde: Keysym = Keysym(0x0000fe6a);
    /// dead_belowbreve
    pub const dead_belowbreve: Keysym = Keysym(0x0000fe6b);
    /// dead_belowdiaeresis
    pub const dead_belowdiaeresis: Keysym = Keysym(0x0000fe6c);
    /// dead_invertedbreve
    pub const dead_invertedbreve: Keysym = Keysym(0x0000fe6d);
    /// dead_belowcomma
    pub const dead_belowcomma: Keysym = Keysym(0x0000fe6e);
    /// dead_currency
    pub const dead_currency: Keysym = Keysym(0x0000fe6f);
    /// AccessX_Enable
    pub const AccessX_Enable: Keysym = Keysym(0x0000fe70);
    /// AccessX_Feedback_Enable
    pub const AccessX_Feedback_Enable: Keysym = Keysym(0x0000fe71);
    /// RepeatKeys_Enable
    pub const RepeatKeys_Enable: Keysym = Keysym(0x0000fe72);
    /// SlowKeys_Enable
    pub const SlowKeys_Enable: Keysym = Keysym(0x0000fe73);
    /// BounceKeys_Enable
    pub const BounceKeys_Enable: Keysym = Keysym(0x0000fe74);
    /// StickyKeys_Enable
    pub const StickyKeys_Enable: Keysym = Keysym(0x0000fe75);
    /// MouseKeys_Enable
    pub const MouseKeys_Enable: Keysym = Keysym(0x0000fe76);
    /// MouseKeys_Accel_Enable
    pub const MouseKeys_Accel_Enable: Keysym = Keysym(0x0000fe77);
    /// Overlay1_Enable
    pub const Overlay1_Enable: Keysym = Keysym(0x0000fe78);
    /// Overlay2_Enable
    pub const Overlay2_Enable: Keysym = Keysym(0x0000fe79);
    /// AudibleBell_Enable
    pub const AudibleBell_Enable: Keysym = Keysym(0x0000fe7a);
    /// dead_a
    pub const dead_a: Keysym = Keysym(0x0000fe80);
    /// dead_A
    pub const dead_A: Keysym = Keysym(0x0000fe81);
    /// dead_e
    pub const dead_e: Keysym = Keysym(0x0000fe82);
    /// dead_E
    pub const dead_E: Keysym = Keysym(0x0000fe83);
    /// dead_i
    pub const dead_i: Keysym = Keysym(0x0000fe84);
    /// dead_I
    pub const dead_I: Keysym = Keysym(0x0000fe85);
    /// dead_o
    pub const dead_o: Keysym = Keysym(0x0000fe86);
    /// dead_O
    pub const dead_O: Keysym = Keysym(0x0000fe87);
    /// dead_u
    pub const dead_u: Keysym = Keysym(0x0000fe88);
    /// dead_U
    pub const dead_U: Keysym = Keysym(0x0000fe89);
    /// dead_small_schwa
    pub const dead_small_schwa: Keysym = Keysym(0x0000fe8a);
    /// dead_schwa
    pub const dead_schwa: Keysym = Keysym(0x0000fe8a);
    /// dead_capital_schwa
    pub const dead_capital_schwa: Keysym = Keysym(0x0000fe8b);
    /// dead_SCHWA
    pub const dead_SCHWA: Keysym = Keysym(0x0000fe8b);
    /// dead_greek
    pub const dead_greek: Keysym = Keysym(0x0000fe8c);
    /// dead_hamza
    pub const dead_hamza: Keysym = Keysym(0x0000fe8d);
    /// dead_lowline
    pub const dead_lowline: Keysym = Keysym(0x0000fe90);
    /// dead_aboveverticalline
    pub const dead_aboveverticalline: Keysym = Keysym(0x0000fe91);
    /// dead_belowverticalline
    pub const dead_belowverticalline: Keysym = Keysym(0x0000fe92);
    /// dead_longsolidusoverlay
    pub const dead_longsolidusoverlay: Keysym = Keysym(0x0000fe93);
    /// ch
    pub const ch: Keysym = Keysym(0x0000fea0);
    /// Ch
    pub const Ch: Keysym = Keysym(0x0000fea1);
    /// CH
    pub const CH: Keysym = Keysym(0x0000fea2);
    /// c_h
    pub const c_h: Keysym = Keysym(0x0000fea3);
    /// C_h
    pub const C_h: Keysym = Keysym(0x0000fea4);
    /// C_H
    pub const C_H: Keysym = Keysym(0x0000fea5);
    /// First_Virtual_Screen
    pub const First_Virtual_Screen: Keysym = Keysym(0x0000fed0);
    /// Prev_Virtual_Screen
    pub const Prev_Virtual_Screen: Keysym = Keysym(0x0000fed1);
    /// Next_Virtual_Screen
    pub const Next_Virtual_Screen: Keysym = Keysym(0x0000fed2);
    /// Last_Virtual_Screen
    pub const Last_Virtual_Screen: Keysym = Keysym(0x0000fed4);
    /// Terminate_Server
    pub const Terminate_Server: Keysym = Keysym(0x0000fed5);
    /// Pointer_Left
    pub const Pointer_Left: Keysym = Keysym(0x0000fee0);
    /// Pointer_Right
    pub const Pointer_Right: Keysym = Keysym(0x0000fee1);
    /// Pointer_Up
    pub const Pointer_Up: Keysym = Keysym(0x0000fee2);
    /// Pointer_Down
    pub const Pointer_Down: Keysym = Keysym(0x0000fee3);
    /// Pointer_UpLeft
    pub const Pointer_UpLeft: Keysym = Keysym(0x0000fee4);
    /// Pointer_UpRight
    pub const Pointer_UpRight: Keysym = Keysym(0x0000fee5);
    /// Pointer_DownLeft
    pub const Pointer_DownLeft: Keysym = Keysym(0x0000fee6);
    /// Pointer_DownRight
    pub const Pointer_DownRight: Keysym = Keysym(0x0000fee7);
    /// Pointer_Button_Dflt
    pub const Pointer_Button_Dflt: Keysym = Keysym(0x0000fee8);
    /// Pointer_Button1
    pub const Pointer_Button1: Keysym = Keysym(0x0000fee9);
    /// Pointer_Button2
    pub const Pointer_Button2: Keysym = Keysym(0x0000feea);
    /// Pointer_Button3
    pub const Pointer_Button3: Keysym = Keysym(0x0000feeb);
    /// Pointer_Button4
    pub const Pointer_Button4: Keysym = Keysym(0x0000feec);
    /// Pointer_Button5
    pub const Pointer_Button5: Keysym = Keysym(0x0000feed);
    /// Pointer_DblClick_Dflt
    pub const Pointer_DblClick_Dflt: Keysym = Keysym(0x0000feee);
    /// Pointer_DblClick1
    pub const Pointer_DblClick1: Keysym = Keysym(0x0000feef);
    /// Pointer_DblClick2
    pub const Pointer_DblClick2: Keysym = Keysym(0x0000fef0);
    /// Pointer_DblClick3
    pub const Pointer_DblClick3: Keysym = Keysym(0x0000fef1);
    /// Pointer_DblClick4
    pub const Pointer_DblClick4: Keysym = Keysym(0x0000fef2);
    /// Pointer_DblClick5
    pub const Pointer_DblClick5: Keysym = Keysym(0x0000fef3);
    /// Pointer_Drag_Dflt
    pub const Pointer_Drag_Dflt: Keysym = Keysym(0x0000fef4);
    /// Pointer_Drag1
    pub const Pointer_Drag1: Keysym = Keysym(0x0000fef5);
    /// Pointer_Drag2
    pub const Pointer_Drag2: Keysym = Keysym(0x0000fef6);
    /// Pointer_Drag3
    pub const Pointer_Drag3: Keysym = Keysym(0x0000fef7);
    /// Pointer_Drag4
    pub const Pointer_Drag4: Keysym = Keysym(0x0000fef8);
    /// Pointer_EnableKeys
    pub const Pointer_EnableKeys: Keysym = Keysym(0x0000fef9);
    /// Pointer_Accelerate
    pub const Pointer_Accelerate: Keysym = Keysym(0x0000fefa);
    /// Pointer_DfltBtnNext
    pub const Pointer_DfltBtnNext: Keysym = Keysym(0x0000fefb);
    /// Pointer_DfltBtnPrev
    pub const Pointer_DfltBtnPrev: Keysym = Keysym(0x0000fefc);
    /// Pointer_Drag5
    pub const Pointer_Drag5: Keysym = Keysym(0x0000fefd);
    /// BackSpace
    pub const BackSpace: Keysym = Keysym(0x0000ff08);
    /// Tab
    pub const Tab: Keysym = Keysym(0x0000ff09);
    /// Linefeed
    pub const Linefeed: Keysym = Keysym(0x0000ff0a);
    /// Clear
    pub const Clear: Keysym = Keysym(0x0000ff0b);
    /// Return
    pub const Return: Keysym = Keysym(0x0000ff0d);
    /// Pause
    pub const Pause: Keysym = Keysym(0x0000ff13);
    /// Scroll_Lock
    pub const Scroll_Lock: Keysym = Keysym(0x0000ff14);
    /// Sys_Req
    pub const Sys_Req: Keysym = Keysym(0x0000ff15);
    /// Escape
    pub const Escape: Keysym = Keysym(0x0000ff1b);
    /// Multi_key
    pub const Multi_key: Keysym = Keysym(0x0000ff20);
    /// SunCompose
    pub const SunCompose: Keysym = Keysym(0x0000ff20);
    /// Kanji
    pub const Kanji: Keysym = Keysym(0x0000ff21);
    /// Muhenkan
    pub const Muhenkan: Keysym = Keysym(0x0000ff22);
    /// Henkan_Mode
    pub const Henkan_Mode: Keysym = Keysym(0x0000ff23);
    /// Henkan
    pub const Henkan: Keysym = Keysym(0x0000ff23);
    /// Romaji
    pub const Romaji: Keysym = Keysym(0x0000ff24);
    /// Hiragana
    pub const Hiragana: Keysym = Keysym(0x0000ff25);
    /// Katakana
    pub const Katakana: Keysym = Keysym(0x0000ff26);
    /// Hiragana_Katakana
    pub const Hiragana_Katakana: Keysym = Keysym(0x0000ff27);
    /// Zenkaku
    pub const Zenkaku: Keysym = Keysym(0x0000ff28);
    /// Hankaku
    pub const Hankaku: Keysym = Keysym(0x0000ff29);
    /// Zenkaku_Hankaku
    pub const Zenkaku_Hankaku: Keysym = Keysym(0x0000ff2a);
    /// Touroku
    pub const Touroku: Keysym = Keysym(0x0000ff2b);
    /// Massyo
    pub const Massyo: Keysym = Keysym(0x0000ff2c);
    /// Kana_Lock
    pub const Kana_Lock: Keysym = Keysym(0x0000ff2d);
    /// Kana_Shift
    pub const Kana_Shift: Keysym = Keysym(0x0000ff2e);
    /// Eisu_Shift
    pub const Eisu_Shift: Keysym = Keysym(0x0000ff2f);
    /// Eisu_toggle
    pub const Eisu_toggle: Keysym = Keysym(0x0000ff30);
    /// Hangul
    pub const Hangul: Keysym = Keysym(0x0000ff31);
    /// Hangul_Start
    pub const Hangul_Start: Keysym = Keysym(0x0000ff32);
    /// Hangul_End
    pub const Hangul_End: Keysym = Keysym(0x0000ff33);
    /// Hangul_Hanja
    pub const Hangul_Hanja: Keysym = Keysym(0x0000ff34);
    /// Hangul_Jamo
    pub const Hangul_Jamo: Keysym = Keysym(0x0000ff35);
    /// Hangul_Romaja
    pub const Hangul_Romaja: Keysym = Keysym(0x0000ff36);
    /// Codeinput
    pub const Codeinput: Keysym = Keysym(0x0000ff37);
    /// Kanji_Bangou
    pub const Kanji_Bangou: Keysym = Keysym(0x0000ff37);
    /// Hangul_Codeinput
    pub const Hangul_Codeinput: Keysym = Keysym(0x0000ff37);
    /// Hangul_Jeonja
    pub const Hangul_Jeonja: Keysym = Keysym(0x0000ff38);
    /// Hangul_Banja
    pub const Hangul_Banja: Keysym = Keysym(0x0000ff39);
    /// Hangul_PreHanja
    pub const Hangul_PreHanja: Keysym = Keysym(0x0000ff3a);
    /// Hangul_PostHanja
    pub const Hangul_PostHanja: Keysym = Keysym(0x0000ff3b);
    /// SingleCandidate
    pub const SingleCandidate: Keysym = Keysym(0x0000ff3c);
    /// Hangul_SingleCandidate
    pub const Hangul_SingleCandidate: Keysym = Keysym(0x0000ff3c);
    /// MultipleCandidate
    pub const MultipleCandidate: Keysym = Keysym(0x0000ff3d);
    /// Zen_Koho
    pub const Zen_Koho: Keysym = Keysym(0x0000ff3d);
    /// Hangul_MultipleCandidate
    pub const Hangul_MultipleCandidate: Keysym = Keysym(0x0000ff3d);
    /// PreviousCandidate
    pub const PreviousCandidate: Keysym = Keysym(0x0000ff3e);
    /// Mae_Koho
    pub const Mae_Koho: Keysym = Keysym(0x0000ff3e);
    /// Hangul_PreviousCandidate
    pub const Hangul_PreviousCandidate: Keysym = Keysym(0x0000ff3e);
    /// Hangul_Special
    pub const Hangul_Special: Keysym = Keysym(0x0000ff3f);
    /// Home
    pub const Home: Keysym = Keysym(0x0000ff50);
    /// Left
    pub const Left: Keysym = Keysym(0x0000ff51);
    /// Up
    pub const Up: Keysym = Keysym(0x0000ff52);
    /// Right
    pub const Right: Keysym = Keysym(0x0000ff53);
    /// Down
    pub const Down: Keysym = Keysym(0x0000ff54);
    /// Prior
    pub const Prior: Keysym = Keysym(0x0000ff55);
    /// Page_Up
    pub const Page_Up: Keysym = Keysym(0x0000ff55);
    /// SunPageUp
    pub const SunPageUp: Keysym = Keysym(0x0000ff55);
    /// Next
    pub const Next: Keysym = Keysym(0x0000ff56);
    /// Page_Down
    pub const Page_Down: Keysym = Keysym(0x0000ff56);
    /// SunPageDown
    pub const SunPageDown: Keysym = Keysym(0x0000ff56);
    /// End
    pub const End: Keysym = Keysym(0x0000ff57);
    /// Begin
    pub const Begin: Keysym = Keysym(0x0000ff58);
    /// Select
    pub const Select: Keysym = Keysym(0x0000ff60);
    /// Print
    pub const Print: Keysym = Keysym(0x0000ff61);
    /// SunPrint_Screen
    pub const SunPrint_Screen: Keysym = Keysym(0x0000ff61);
    /// Execute
    pub const Execute: Keysym = Keysym(0x0000ff62);
    /// Insert
    pub const Insert: Keysym = Keysym(0x0000ff63);
    /// Undo
    pub const Undo: Keysym = Keysym(0x0000ff65);
    /// SunUndo
    pub const SunUndo: Keysym = Keysym(0x0000ff65);
    /// Redo
    pub const Redo: Keysym = Keysym(0x0000ff66);
    /// SunAgain
    pub const SunAgain: Keysym = Keysym(0x0000ff66);
    /// Menu
    pub const Menu: Keysym = Keysym(0x0000ff67);
    /// Find
    pub const Find: Keysym = Keysym(0x0000ff68);
    /// SunFind
    pub const SunFind: Keysym = Keysym(0x0000ff68);
    /// Cancel
    pub const Cancel: Keysym = Keysym(0x0000ff69);
    /// SunStop
    pub const SunStop: Keysym = Keysym(0x0000ff69);
    /// Help
    pub const Help: Keysym = Keysym(0x0000ff6a);
    /// Break
    pub const Break: Keysym = Keysym(0x0000ff6b);
    /// Mode_switch
    pub const Mode_switch: Keysym = Keysym(0x0000ff7e);
    /// script_switch
    pub const script_switch: Keysym = Keysym(0x0000ff7e);
    /// ISO_Group_Shift
    pub const ISO_Group_Shift: Keysym = Keysym(0x0000ff7e);
    /// kana_switch
    pub const kana_switch: Keysym = Keysym(0x0000ff7e);
    /// Arabic_switch
    pub const Arabic_switch: Keysym = Keysym(0x0000ff7e);
    /// Greek_switch
    pub const Greek_switch: Keysym = Keysym(0x0000ff7e);
    /// Hebrew_switch
    pub const Hebrew_switch: Keysym = Keysym(0x0000ff7e);
    /// Hangul_switch
    pub const Hangul_switch: Keysym = Keysym(0x0000ff7e);
    /// SunAltGraph
    pub const SunAltGraph: Keysym = Keysym(0x0000ff7e);
    /// Num_Lock
    pub const Num_Lock: Keysym = Keysym(0x0000ff7f);
    /// KP_Space
    pub const KP_Space: Keysym = Keysym(0x0000ff80);
    /// KP_Tab
    pub const KP_Tab: Keysym = Keysym(0x0000ff89);
    /// KP_Enter
    pub const KP_Enter: Keysym = Keysym(0x0000ff8d);
    /// KP_F1
    pub const KP_F1: Keysym = Keysym(0x0000ff91);
    /// KP_F2
    pub const KP_F2: Keysym = Keysym(0x0000ff92);
    /// KP_F3
    pub const KP_F3: Keysym = Keysym(0x0000ff93);
    /// KP_F4
    pub const KP_F4: Keysym = Keysym(0x0000ff94);
    /// KP_Home
    pub const KP_Home: Keysym = Keysym(0x0000ff95);
    /// KP_Left
    pub const KP_Left: Keysym = Keysym(0x0000ff96);
    /// KP_Up
    pub const KP_Up: Keysym = Keysym(0x0000ff97);
    /// KP_Right
    pub const KP_Right: Keysym = Keysym(0x0000ff98);
    /// KP_Down
    pub const KP_Down: Keysym = Keysym(0x0000ff99);
    /// KP_Prior
    pub const KP_Prior: Keysym = Keysym(0x0000ff9a);
    /// KP_Page_Up
    pub const KP_Page_Up: Keysym = Keysym(0x0000ff9a);
    /// KP_Next
    pub const KP_Next: Keysym = Keysym(0x0000ff9b);
    /// KP_Page_Down
    pub const KP_Page_Down: Keysym = Keysym(0x0000ff9b);
    /// KP_End
    pub const KP_End: Keysym = Keysym(0x0000ff9c);
    /// KP_Begin
    pub const KP_Begin: Keysym = Keysym(0x0000ff9d);
    /// KP_Insert
    pub const KP_Insert: Keysym = Keysym(0x0000ff9e);
    /// KP_Delete
    pub const KP_Delete: Keysym = Keysym(0x0000ff9f);
    /// KP_Multiply
    pub const KP_Multiply: Keysym = Keysym(0x0000ffaa);
    /// KP_Add
    pub const KP_Add: Keysym = Keysym(0x0000ffab);
    /// KP_Separator
    pub const KP_Separator: Keysym = Keysym(0x0000ffac);
    /// KP_Subtract
    pub const KP_Subtract: Keysym = Keysym(0x0000ffad);
    /// KP_Decimal
    pub const KP_Decimal: Keysym = Keysym(0x0000ffae);
    /// KP_Divide
    pub const KP_Divide: Keysym = Keysym(0x0000ffaf);
    /// KP_0
    pub const KP_0: Keysym = Keysym(0x0000ffb0);
    /// KP_1
    pub const KP_1: Keysym = Keysym(0x0000ffb1);
    /// KP_2
    pub const KP_2: Keysym = Keysym(0x0000ffb2);
    /// KP_3
    pub const KP_3: Keysym = Keysym(0x0000ffb3);
    /// KP_4
    pub const KP_4: Keysym = Keysym(0x0000ffb4);
    /// KP_5
    pub const KP_5: Keysym = Keysym(0x0000ffb5);
    /// KP_6
    pub const KP_6: Keysym = Keysym(0x0000ffb6);
    /// KP_7
    pub const KP_7: Keysym = Keysym(0x0000ffb7);
    /// KP_8
    pub const KP_8: Keysym = Keysym(0x0000ffb8);
    /// KP_9
    pub const KP_9: Keysym = Keysym(0x0000ffb9);
    /// KP_Equal
    pub const KP_Equal: Keysym = Keysym(0x0000ffbd);
    /// F1
    pub const F1: Keysym = Keysym(0x0000ffbe);
    /// F2
    pub const F2: Keysym = Keysym(0x0000ffbf);
    /// F3
    pub const F3: Keysym = Keysym(0x0000ffc0);
    /// F4
    pub const F4: Keysym = Keysym(0x0000ffc1);
    /// F5
    pub const F5: Keysym = Keysym(0x0000ffc2);
    /// F6
    pub const F6: Keysym = Keysym(0x0000ffc3);
    /// F7
    pub const F7: Keysym = Keysym(0x0000ffc4);
    /// F8
    pub const F8: Keysym = Keysym(0x0000ffc5);
    /// F9
    pub const F9: Keysym = Keysym(0x0000ffc6);
    /// F10
    pub const F10: Keysym = Keysym(0x0000ffc7);
    /// F11
    pub const F11: Keysym = Keysym(0x0000ffc8);
    /// L1
    pub const L1: Keysym = Keysym(0x0000ffc8);
    /// F12
    pub const F12: Keysym = Keysym(0x0000ffc9);
    /// L2
    pub const L2: Keysym = Keysym(0x0000ffc9);
    /// F13
    pub const F13: Keysym = Keysym(0x0000ffca);
    /// L3
    pub const L3: Keysym = Keysym(0x0000ffca);
    /// F14
    pub const F14: Keysym = Keysym(0x0000ffcb);
    /// L4
    pub const L4: Keysym = Keysym(0x0000ffcb);
    /// F15
    pub const F15: Keysym = Keysym(0x0000ffcc);
    /// L5
    pub const L5: Keysym = Keysym(0x0000ffcc);
    /// F16
    pub const F16: Keysym = Keysym(0x0000ffcd);
    /// L6
    pub const L6: Keysym = Keysym(0x0000ffcd);
    /// F17
    pub const F17: Keysym = Keysym(0x0000ffce);
    /// L7
    pub const L7: Keysym = Keysym(0x0000ffce);
    /// F18
    pub const F18: Keysym = Keysym(0x0000ffcf);
    /// L8
    pub const L8: Keysym = Keysym(0x0000ffcf);
    /// F19
    pub const F19: Keysym = Keysym(0x0000ffd0);
    /// L9
    pub const L9: Keysym = Keysym(0x0000ffd0);
    /// F20
    pub const F20: Keysym = Keysym(0x0000ffd1);
    /// L10
    pub const L10: Keysym = Keysym(0x0000ffd1);
    /// F21
    pub const F21: Keysym = Keysym(0x0000ffd2);
    /// R1
    pub const R1: Keysym = Keysym(0x0000ffd2);
    /// F22
    pub const F22: Keysym = Keysym(0x0000ffd3);
    /// R2
    pub const R2: Keysym = Keysym(0x0000ffd3);
    /// F23
    pub const F23: Keysym = Keysym(0x0000ffd4);
    /// R3
    pub const R3: Keysym = Keysym(0x0000ffd4);
    /// F24
    pub const F24: Keysym = Keysym(0x0000ffd5);
    /// R4
    pub const R4: Keysym = Keysym(0x0000ffd5);
    /// F25
    pub const F25: Keysym = Keysym(0x0000ffd6);
    /// R5
    pub const R5: Keysym = Keysym(0x0000ffd6);
    /// F26
    pub const F26: Keysym = Keysym(0x0000ffd7);
    /// R6
    pub const R6: Keysym = Keysym(0x0000ffd7);
    /// F27
    pub const F27: Keysym = Keysym(0x0000ffd8);
    /// R7
    pub const R7: Keysym = Keysym(0x0000ffd8);
    /// F28
    pub const F28: Keysym = Keysym(0x0000ffd9);
    /// R8
    pub const R8: Keysym = Keysym(0x0000ffd9);
    /// F29
    pub const F29: Keysym = Keysym(0x0000ffda);
    /// R9
    pub const R9: Keysym = Keysym(0x0000ffda);
    /// F30
    pub const F30: Keysym = Keysym(0x0000ffdb);
    /// R10
    pub const R10: Keysym = Keysym(0x0000ffdb);
    /// F31
    pub const F31: Keysym = Keysym(0x0000ffdc);
    /// R11
    pub const R11: Keysym = Keysym(0x0000ffdc);
    /// F32
    pub const F32: Keysym = Keysym(0x0000ffdd);
    /// R12
    pub const R12: Keysym = Keysym(0x0000ffdd);
    /// F33
    pub const F33: Keysym = Keysym(0x0000ffde);
    /// R13
    pub const R13: Keysym = Keysym(0x0000ffde);
    /// F34
    pub const F34: Keysym = Keysym(0x0000ffdf);
    /// R14
    pub const R14: Keysym = Keysym(0x0000ffdf);
    /// F35
    pub const F35: Keysym = Keysym(0x0000ffe0);
    /// R15
    pub const R15: Keysym = Keysym(0x0000ffe0);
    /// Shift_L
    pub const Shift_L: Keysym = Keysym(0x0000ffe1);
    /// Shift_R
    pub const Shift_R: Keysym = Keysym(0x0000ffe2);
    /// Control_L
    pub const Control_L: Keysym = Keysym(0x0000ffe3);
    /// Control_R
    pub const Control_R: Keysym = Keysym(0x0000ffe4);
    /// Caps_Lock
    pub const Caps_Lock: Keysym = Keysym(0x0000ffe5);
    /// Shift_Lock
    pub const Shift_Lock: Keysym = Keysym(0x0000ffe6);
    /// Meta_L
    pub const Meta_L: Keysym = Keysym(0x0000ffe7);
    /// Meta_R
    pub const Meta_R: Keysym = Keysym(0x0000ffe8);
    /// Alt_L
    pub const Alt_L: Keysym = Keysym(0x0000ffe9);
    /// Alt_R
    pub const Alt_R: Keysym = Keysym(0x0000ffea);
    /// Super_L
    pub const Super_L: Keysym = Keysym(0x0000ffeb);
    /// Super_R
    pub const Super_R: Keysym = Keysym(0x0000ffec);
    /// Hyper_L
    pub const Hyper_L: Keysym = Keysym(0x0000ffed);
    /// Hyper_R
    pub const Hyper_R: Keysym = Keysym(0x0000ffee);
    /// braille_dot_1
    pub const braille_dot_1: Keysym = Keysym(0x0000fff1);
    /// braille_dot_2
    pub const braille_dot_2: Keysym = Keysym(0x0000fff2);
    /// braille_dot_3
    pub const braille_dot_3: Keysym = Keysym(0x0000fff3);
    /// braille_dot_4
    pub const braille_dot_4: Keysym = Keysym(0x0000fff4);
    /// braille_dot_5
    pub const braille_dot_5: Keysym = Keysym(0x0000fff5);
    /// braille_dot_6
    pub const braille_dot_6: Keysym = Keysym(0x0000fff6);
    /// braille_dot_7
    pub const braille_dot_7: Keysym = Keysym(0x0000fff7);
    /// braille_dot_8
    pub const braille_dot_8: Keysym = Keysym(0x0000fff8);
    /// braille_dot_9
    pub const braille_dot_9: Keysym = Keysym(0x0000fff9);
    /// braille_dot_10
    pub const braille_dot_10: Keysym = Keysym(0x0000fffa);
    /// Delete
    pub const Delete: Keysym = Keysym(0x0000ffff);
    /// VoidSymbol
    pub const VoidSymbol: Keysym = Keysym(0x00ffffff);
    /// Ibreve
    pub const Ibreve: Keysym = Keysym(0x0100012c);
    /// ibreve
    pub const ibreve: Keysym = Keysym(0x0100012d);
    /// Wcircumflex
    pub const Wcircumflex: Keysym = Keysym(0x01000174);
    /// wcircumflex
    pub const wcircumflex: Keysym = Keysym(0x01000175);
    /// Ycircumflex
    pub const Ycircumflex: Keysym = Keysym(0x01000176);
    /// ycircumflex
    pub const ycircumflex: Keysym = Keysym(0x01000177);
    /// SCHWA
    pub const SCHWA: Keysym = Keysym(0x0100018f);
    /// Obarred
    pub const Obarred: Keysym = Keysym(0x0100019f);
    /// Ohorn
    pub const Ohorn: Keysym = Keysym(0x010001a0);
    /// ohorn
    pub const ohorn: Keysym = Keysym(0x010001a1);
    /// Uhorn
    pub const Uhorn: Keysym = Keysym(0x010001af);
    /// uhorn
    pub const uhorn: Keysym = Keysym(0x010001b0);
    /// Zstroke
    pub const Zstroke: Keysym = Keysym(0x010001b5);
    /// zstroke
    pub const zstroke: Keysym = Keysym(0x010001b6);
    /// EZH
    pub const EZH: Keysym = Keysym(0x010001b7);
    /// Ocaron
    pub const Ocaron: Keysym = Keysym(0x010001d1);
    /// ocaron
    pub const ocaron: Keysym = Keysym(0x010001d2);
    /// Gcaron
    pub const Gcaron: Keysym = Keysym(0x010001e6);
    /// gcaron
    pub const gcaron: Keysym = Keysym(0x010001e7);
    /// schwa
    pub const schwa: Keysym = Keysym(0x01000259);
    /// obarred
    pub const obarred: Keysym = Keysym(0x01000275);
    /// ezh
    pub const ezh: Keysym = Keysym(0x01000292);
    /// combining_grave
    pub const combining_grave: Keysym = Keysym(0x01000300);
    /// combining_acute
    pub const combining_acute: Keysym = Keysym(0x01000301);
    /// combining_tilde
    pub const combining_tilde: Keysym = Keysym(0x01000303);
    /// combining_hook
    pub const combining_hook: Keysym = Keysym(0x01000309);
    /// combining_belowdot
    pub const combining_belowdot: Keysym = Keysym(0x01000323);
    /// Cyrillic_GHE_bar
    pub const Cyrillic_GHE_bar: Keysym = Keysym(0x01000492);
    /// Cyrillic_ghe_bar
    pub const Cyrillic_ghe_bar: Keysym = Keysym(0x01000493);
    /// Cyrillic_ZHE_descender
    pub const Cyrillic_ZHE_descender: Keysym = Keysym(0x01000496);
    /// Cyrillic_zhe_descender
    pub const Cyrillic_zhe_descender: Keysym = Keysym(0x01000497);
    /// Cyrillic_KA_descender
    pub const Cyrillic_KA_descender: Keysym = Keysym(0x0100049a);
    /// Cyrillic_ka_descender
    pub const Cyrillic_ka_descender: Keysym = Keysym(0x0100049b);
    /// Cyrillic_KA_vertstroke
    pub const Cyrillic_KA_vertstroke: Keysym = Keysym(0x0100049c);
    /// Cyrillic_ka_vertstroke
    pub const Cyrillic_ka_vertstroke: Keysym = Keysym(0x0100049d);
    /// Cyrillic_EN_descender
    pub const Cyrillic_EN_descender: Keysym = Keysym(0x010004a2);
    /// Cyrillic_en_descender
    pub const Cyrillic_en_descender: Keysym = Keysym(0x010004a3);
    /// Cyrillic_U_straight
    pub const Cyrillic_U_straight: Keysym = Keysym(0x010004ae);
    /// Cyrillic_u_straight
    pub const Cyrillic_u_straight: Keysym = Keysym(0x010004af);
    /// Cyrillic_U_straight_bar
    pub const Cyrillic_U_straight_bar: Keysym = Keysym(0x010004b0);
    /// Cyrillic_u_straight_bar
    pub const Cyrillic_u_straight_bar: Keysym = Keysym(0x010004b1);
    /// Cyrillic_HA_descender
    pub const Cyrillic_HA_descender: Keysym = Keysym(0x010004b2);
    /// Cyrillic_ha_descender
    pub const Cyrillic_ha_descender: Keysym = Keysym(0x010004b3);
    /// Cyrillic_CHE_descender
    pub const Cyrillic_CHE_descender: Keysym = Keysym(0x010004b6);
    /// Cyrillic_che_descender
    pub const Cyrillic_che_descender: Keysym = Keysym(0x010004b7);
    /// Cyrillic_CHE_vertstroke
    pub const Cyrillic_CHE_vertstroke: Keysym = Keysym(0x010004b8);
    /// Cyrillic_che_vertstroke
    pub const Cyrillic_che_vertstroke: Keysym = Keysym(0x010004b9);
    /// Cyrillic_SHHA
    pub const Cyrillic_SHHA: Keysym = Keysym(0x010004ba);
    /// Cyrillic_shha
    pub const Cyrillic_shha: Keysym = Keysym(0x010004bb);
    /// Cyrillic_SCHWA
    pub const Cyrillic_SCHWA: Keysym = Keysym(0x010004d8);
    /// Cyrillic_schwa
    pub const Cyrillic_schwa: Keysym = Keysym(0x010004d9);
    /// Cyrillic_I_macron
    pub const Cyrillic_I_macron: Keysym = Keysym(0x010004e2);
    /// Cyrillic_i_macron
    pub const Cyrillic_i_macron: Keysym = Keysym(0x010004e3);
    /// Cyrillic_O_bar
    pub const Cyrillic_O_bar: Keysym = Keysym(0x010004e8);
    /// Cyrillic_o_bar
    pub const Cyrillic_o_bar: Keysym = Keysym(0x010004e9);
    /// Cyrillic_U_macron
    pub const Cyrillic_U_macron: Keysym = Keysym(0x010004ee);
    /// Cyrillic_u_macron
    pub const Cyrillic_u_macron: Keysym = Keysym(0x010004ef);
    /// Armenian_AYB
    pub const Armenian_AYB: Keysym = Keysym(0x01000531);
    /// Armenian_BEN
    pub const Armenian_BEN: Keysym = Keysym(0x01000532);
    /// Armenian_GIM
    pub const Armenian_GIM: Keysym = Keysym(0x01000533);
    /// Armenian_DA
    pub const Armenian_DA: Keysym = Keysym(0x01000534);
    /// Armenian_YECH
    pub const Armenian_YECH: Keysym = Keysym(0x01000535);
    /// Armenian_ZA
    pub const Armenian_ZA: Keysym = Keysym(0x01000536);
    /// Armenian_E
    pub const Armenian_E: Keysym = Keysym(0x01000537);
    /// Armenian_AT
    pub const Armenian_AT: Keysym = Keysym(0x01000538);
    /// Armenian_TO
    pub const Armenian_TO: Keysym = Keysym(0x01000539);
    /// Armenian_ZHE
    pub const Armenian_ZHE: Keysym = Keysym(0x0100053a);
    /// Armenian_INI
    pub const Armenian_INI: Keysym = Keysym(0x0100053b);
    /// Armenian_LYUN
    pub const Armenian_LYUN: Keysym = Keysym(0x0100053c);
    /// Armenian_KHE
    pub const Armenian_KHE: Keysym = Keysym(0x0100053d);
    /// Armenian_TSA
    pub const Armenian_TSA: Keysym = Keysym(0x0100053e);
    /// Armenian_KEN
    pub const Armenian_KEN: Keysym = Keysym(0x0100053f);
    /// Armenian_HO
    pub const Armenian_HO: Keysym = Keysym(0x01000540);
    /// Armenian_DZA
    pub const Armenian_DZA: Keysym = Keysym(0x01000541);
    /// Armenian_GHAT
    pub const Armenian_GHAT: Keysym = Keysym(0x01000542);
    /// Armenian_TCHE
    pub const Armenian_TCHE: Keysym = Keysym(0x01000543);
    /// Armenian_MEN
    pub const Armenian_MEN: Keysym = Keysym(0x01000544);
    /// Armenian_HI
    pub const Armenian_HI: Keysym = Keysym(0x01000545);
    /// Armenian_NU
    pub const Armenian_NU: Keysym = Keysym(0x01000546);
    /// Armenian_SHA
    pub const Armenian_SHA: Keysym = Keysym(0x01000547);
    /// Armenian_VO
    pub const Armenian_VO: Keysym = Keysym(0x01000548);
    /// Armenian_CHA
    pub const Armenian_CHA: Keysym = Keysym(0x01000549);
    /// Armenian_PE
    pub const Armenian_PE: Keysym = Keysym(0x0100054a);
    /// Armenian_JE
    pub const Armenian_JE: Keysym = Keysym(0x0100054b);
    /// Armenian_RA
    pub const Armenian_RA: Keysym = Keysym(0x0100054c);
    /// Armenian_SE
    pub const Armenian_SE: Keysym = Keysym(0x0100054d);
    /// Armenian_VEV
    pub const Armenian_VEV: Keysym = Keysym(0x0100054e);
    /// Armenian_TYUN
    pub const Armenian_TYUN: Keysym = Keysym(0x0100054f);
    /// Armenian_RE
    pub const Armenian_RE: Keysym = Keysym(0x01000550);
    /// Armenian_TSO
    pub const Armenian_TSO: Keysym = Keysym(0x01000551);
    /// Armenian_VYUN
    pub const Armenian_VYUN: Keysym = Keysym(0x01000552);
    /// Armenian_PYUR
    pub const Armenian_PYUR: Keysym = Keysym(0x01000553);
    /// Armenian_KE
    pub const Armenian_KE: Keysym = Keysym(0x01000554);
    /// Armenian_O
    pub const Armenian_O: Keysym = Keysym(0x01000555);
    /// Armenian_FE
    pub const Armenian_FE: Keysym = Keysym(0x01000556);
    /// Armenian_apostrophe
    pub const Armenian_apostrophe: Keysym = Keysym(0x0100055a);
    /// Armenian_accent
    pub const Armenian_accent: Keysym = Keysym(0x0100055b);
    /// Armenian_shesht
    pub const Armenian_shesht: Keysym = Keysym(0x0100055b);
    /// Armenian_exclam
    pub const Armenian_exclam: Keysym = Keysym(0x0100055c);
    /// Armenian_amanak
    pub const Armenian_amanak: Keysym = Keysym(0x0100055c);
    /// Armenian_separation_mark
    pub const Armenian_separation_mark: Keysym = Keysym(0x0100055d);
    /// Armenian_but
    pub const Armenian_but: Keysym = Keysym(0x0100055d);
    /// Armenian_question
    pub const Armenian_question: Keysym = Keysym(0x0100055e);
    /// Armenian_paruyk
    pub const Armenian_paruyk: Keysym = Keysym(0x0100055e);
    /// Armenian_ayb
    pub const Armenian_ayb: Keysym = Keysym(0x01000561);
    /// Armenian_ben
    pub const Armenian_ben: Keysym = Keysym(0x01000562);
    /// Armenian_gim
    pub const Armenian_gim: Keysym = Keysym(0x01000563);
    /// Armenian_da
    pub const Armenian_da: Keysym = Keysym(0x01000564);
    /// Armenian_yech
    pub const Armenian_yech: Keysym = Keysym(0x01000565);
    /// Armenian_za
    pub const Armenian_za: Keysym = Keysym(0x01000566);
    /// Armenian_e
    pub const Armenian_e: Keysym = Keysym(0x01000567);
    /// Armenian_at
    pub const Armenian_at: Keysym = Keysym(0x01000568);
    /// Armenian_to
    pub const Armenian_to: Keysym = Keysym(0x01000569);
    /// Armenian_zhe
    pub const Armenian_zhe: Keysym = Keysym(0x0100056a);
    /// Armenian_ini
    pub const Armenian_ini: Keysym = Keysym(0x0100056b);
    /// Armenian_lyun
    pub const Armenian_lyun: Keysym = Keysym(0x0100056c);
    /// Armenian_khe
    pub const Armenian_khe: Keysym = Keysym(0x0100056d);
    /// Armenian_tsa
    pub const Armenian_tsa: Keysym = Keysym(0x0100056e);
    /// Armenian_ken
    pub const Armenian_ken: Keysym = Keysym(0x0100056f);
    /// Armenian_ho
    pub const Armenian_ho: Keysym = Keysym(0x01000570);
    /// Armenian_dza
    pub const Armenian_dza: Keysym = Keysym(0x01000571);
    /// Armenian_ghat
    pub const Armenian_ghat: Keysym = Keysym(0x01000572);
    /// Armenian_tche
    pub const Armenian_tche: Keysym = Keysym(0x01000573);
    /// Armenian_men
    pub const Armenian_men: Keysym = Keysym(0x01000574);
    /// Armenian_hi
    pub const Armenian_hi: Keysym = Keysym(0x01000575);
    /// Armenian_nu
    pub const Armenian_nu: Keysym = Keysym(0x01000576);
    /// Armenian_sha
    pub const Armenian_sha: Keysym = Keysym(0x01000577);
    /// Armenian_vo
    pub const Armenian_vo: Keysym = Keysym(0x01000578);
    /// Armenian_cha
    pub const Armenian_cha: Keysym = Keysym(0x01000579);
    /// Armenian_pe
    pub const Armenian_pe: Keysym = Keysym(0x0100057a);
    /// Armenian_je
    pub const Armenian_je: Keysym = Keysym(0x0100057b);
    /// Armenian_ra
    pub const Armenian_ra: Keysym = Keysym(0x0100057c);
    /// Armenian_se
    pub const Armenian_se: Keysym = Keysym(0x0100057d);
    /// Armenian_vev
    pub const Armenian_vev: Keysym = Keysym(0x0100057e);
    /// Armenian_tyun
    pub const Armenian_tyun: Keysym = Keysym(0x0100057f);
    /// Armenian_re
    pub const Armenian_re: Keysym = Keysym(0x01000580);
    /// Armenian_tso
    pub const Armenian_tso: Keysym = Keysym(0x01000581);
    /// Armenian_vyun
    pub const Armenian_vyun: Keysym = Keysym(0x01000582);
    /// Armenian_pyur
    pub const Armenian_pyur: Keysym = Keysym(0x01000583);
    /// Armenian_ke
    pub const Armenian_ke: Keysym = Keysym(0x01000584);
    /// Armenian_o
    pub const Armenian_o: Keysym = Keysym(0x01000585);
    /// Armenian_fe
    pub const Armenian_fe: Keysym = Keysym(0x01000586);
    /// Armenian_ligature_ew
    pub const Armenian_ligature_ew: Keysym = Keysym(0x01000587);
    /// Armenian_full_stop
    pub const Armenian_full_stop: Keysym = Keysym(0x01000589);
    /// Armenian_verjaket
    pub const Armenian_verjaket: Keysym = Keysym(0x01000589);
    /// Armenian_hyphen
    pub const Armenian_hyphen: Keysym = Keysym(0x0100058a);
    /// Armenian_yentamna
    pub const Armenian_yentamna: Keysym = Keysym(0x0100058a);
    /// Arabic_madda_above
    pub const Arabic_madda_above: Keysym = Keysym(0x01000653);
    /// Arabic_hamza_above
    pub const Arabic_hamza_above: Keysym = Keysym(0x01000654);
    /// Arabic_hamza_below
    pub const Arabic_hamza_below: Keysym = Keysym(0x01000655);
    /// Arabic_0
    pub const Arabic_0: Keysym = Keysym(0x01000660);
    /// Arabic_1
    pub const Arabic_1: Keysym = Keysym(0x01000661);
    /// Arabic_2
    pub const Arabic_2: Keysym = Keysym(0x01000662);
    /// Arabic_3
    pub const Arabic_3: Keysym = Keysym(0x01000663);
    /// Arabic_4
    pub const Arabic_4: Keysym = Keysym(0x01000664);
    /// Arabic_5
    pub const Arabic_5: Keysym = Keysym(0x01000665);
    /// Arabic_6
    pub const Arabic_6: Keysym = Keysym(0x01000666);
    /// Arabic_7
    pub const Arabic_7: Keysym = Keysym(0x01000667);
    /// Arabic_8
    pub const Arabic_8: Keysym = Keysym(0x01000668);
    /// Arabic_9
    pub const Arabic_9: Keysym = Keysym(0x01000669);
    /// Arabic_percent
    pub const Arabic_percent: Keysym = Keysym(0x0100066a);
    /// Arabic_superscript_alef
    pub const Arabic_superscript_alef: Keysym = Keysym(0x01000670);
    /// Arabic_tteh
    pub const Arabic_tteh: Keysym = Keysym(0x01000679);
    /// Arabic_peh
    pub const Arabic_peh: Keysym = Keysym(0x0100067e);
    /// Arabic_tcheh
    pub const Arabic_tcheh: Keysym = Keysym(0x01000686);
    /// Arabic_ddal
    pub const Arabic_ddal: Keysym = Keysym(0x01000688);
    /// Arabic_rreh
    pub const Arabic_rreh: Keysym = Keysym(0x01000691);
    /// Arabic_jeh
    pub const Arabic_jeh: Keysym = Keysym(0x01000698);
    /// Arabic_veh
    pub const Arabic_veh: Keysym = Keysym(0x010006a4);
    /// Arabic_keheh
    pub const Arabic_keheh: Keysym = Keysym(0x010006a9);
    /// Arabic_gaf
    pub const Arabic_gaf: Keysym = Keysym(0x010006af);
    /// Arabic_noon_ghunna
    pub const Arabic_noon_ghunna: Keysym = Keysym(0x010006ba);
    /// Arabic_heh_doachashmee
    pub const Arabic_heh_doachashmee: Keysym = Keysym(0x010006be);
    /// Arabic_heh_goal
    pub const Arabic_heh_goal: Keysym = Keysym(0x010006c1);
    /// Farsi_yeh
    pub const Farsi_yeh: Keysym = Keysym(0x010006cc);
    /// Arabic_farsi_yeh
    pub const Arabic_farsi_yeh: Keysym = Keysym(0x010006cc);
    /// Arabic_yeh_baree
    pub const Arabic_yeh_baree: Keysym = Keysym(0x010006d2);
    /// Arabic_fullstop
    pub const Arabic_fullstop: Keysym = Keysym(0x010006d4);
    /// Farsi_0
    pub const Farsi_0: Keysym = Keysym(0x010006f0);
    /// Farsi_1
    pub const Farsi_1: Keysym = Keysym(0x010006f1);
    /// Farsi_2
    pub const Farsi_2: Keysym = Keysym(0x010006f2);
    /// Farsi_3
    pub const Farsi_3: Keysym = Keysym(0x010006f3);
    /// Farsi_4
    pub const Farsi_4: Keysym = Keysym(0x010006f4);
    /// Farsi_5
    pub const Farsi_5: Keysym = Keysym(0x010006f5);
    /// Farsi_6
    pub const Farsi_6: Keysym = Keysym(0x010006f6);
    /// Farsi_7
    pub const Farsi_7: Keysym = Keysym(0x010006f7);
    /// Farsi_8
    pub const Farsi_8: Keysym = Keysym(0x010006f8);
    /// Farsi_9
    pub const Farsi_9: Keysym = Keysym(0x010006f9);
    /// Sinh_ng
    pub const Sinh_ng: Keysym = Keysym(0x01000d82);
    /// Sinh_h2
    pub const Sinh_h2: Keysym = Keysym(0x01000d83);
    /// Sinh_a
    pub const Sinh_a: Keysym = Keysym(0x01000d85);
    /// Sinh_aa
    pub const Sinh_aa: Keysym = Keysym(0x01000d86);
    /// Sinh_ae
    pub const Sinh_ae: Keysym = Keysym(0x01000d87);
    /// Sinh_aee
    pub const Sinh_aee: Keysym = Keysym(0x01000d88);
    /// Sinh_i
    pub const Sinh_i: Keysym = Keysym(0x01000d89);
    /// Sinh_ii
    pub const Sinh_ii: Keysym = Keysym(0x01000d8a);
    /// Sinh_u
    pub const Sinh_u: Keysym = Keysym(0x01000d8b);
    /// Sinh_uu
    pub const Sinh_uu: Keysym = Keysym(0x01000d8c);
    /// Sinh_ri
    pub const Sinh_ri: Keysym = Keysym(0x01000d8d);
    /// Sinh_rii
    pub const Sinh_rii: Keysym = Keysym(0x01000d8e);
    /// Sinh_lu
    pub const Sinh_lu: Keysym = Keysym(0x01000d8f);
    /// Sinh_luu
    pub const Sinh_luu: Keysym = Keysym(0x01000d90);
    /// Sinh_e
    pub const Sinh_e: Keysym = Keysym(0x01000d91);
    /// Sinh_ee
    pub const Sinh_ee: Keysym = Keysym(0x01000d92);
    /// Sinh_ai
    pub const Sinh_ai: Keysym = Keysym(0x01000d93);
    /// Sinh_o
    pub const Sinh_o: Keysym = Keysym(0x01000d94);
    /// Sinh_oo
    pub const Sinh_oo: Keysym = Keysym(0x01000d95);
    /// Sinh_au
    pub const Sinh_au: Keysym = Keysym(0x01000d96);
    /// Sinh_ka
    pub const Sinh_ka: Keysym = Keysym(0x01000d9a);
    /// Sinh_kha
    pub const Sinh_kha: Keysym = Keysym(0x01000d9b);
    /// Sinh_ga
    pub const Sinh_ga: Keysym = Keysym(0x01000d9c);
    /// Sinh_gha
    pub const Sinh_gha: Keysym = Keysym(0x01000d9d);
    /// Sinh_ng2
    pub const Sinh_ng2: Keysym = Keysym(0x01000d9e);
    /// Sinh_nga
    pub const Sinh_nga: Keysym = Keysym(0x01000d9f);
    /// Sinh_ca
    pub const Sinh_ca: Keysym = Keysym(0x01000da0);
    /// Sinh_cha
    pub const Sinh_cha: Keysym = Keysym(0x01000da1);
    /// Sinh_ja
    pub const Sinh_ja: Keysym = Keysym(0x01000da2);
    /// Sinh_jha
    pub const Sinh_jha: Keysym = Keysym(0x01000da3);
    /// Sinh_nya
    pub const Sinh_nya: Keysym = Keysym(0x01000da4);
    /// Sinh_jnya
    pub const Sinh_jnya: Keysym = Keysym(0x01000da5);
    /// Sinh_nja
    pub const Sinh_nja: Keysym = Keysym(0x01000da6);
    /// Sinh_tta
    pub const Sinh_tta: Keysym = Keysym(0x01000da7);
    /// Sinh_ttha
    pub const Sinh_ttha: Keysym = Keysym(0x01000da8);
    /// Sinh_dda
    pub const Sinh_dda: Keysym = Keysym(0x01000da9);
    /// Sinh_ddha
    pub const Sinh_ddha: Keysym = Keysym(0x01000daa);
    /// Sinh_nna
    pub const Sinh_nna: Keysym = Keysym(0x01000dab);
    /// Sinh_ndda
    pub const Sinh_ndda: Keysym = Keysym(0x01000dac);
    /// Sinh_tha
    pub const Sinh_tha: Keysym = Keysym(0x01000dad);
    /// Sinh_thha
    pub const Sinh_thha: Keysym = Keysym(0x01000dae);
    /// Sinh_dha
    pub const Sinh_dha: Keysym = Keysym(0x01000daf);
    /// Sinh_dhha
    pub const Sinh_dhha: Keysym = Keysym(0x01000db0);
    /// Sinh_na
    pub const Sinh_na: Keysym = Keysym(0x01000db1);
    /// Sinh_ndha
    pub const Sinh_ndha: Keysym = Keysym(0x01000db3);
    /// Sinh_pa
    pub const Sinh_pa: Keysym = Keysym(0x01000db4);
    /// Sinh_pha
    pub const Sinh_pha: Keysym = Keysym(0x01000db5);
    /// Sinh_ba
    pub const Sinh_ba: Keysym = Keysym(0x01000db6);
    /// Sinh_bha
    pub const Sinh_bha: Keysym = Keysym(0x01000db7);
    /// Sinh_ma
    pub const Sinh_ma: Keysym = Keysym(0x01000db8);
    /// Sinh_mba
    pub const Sinh_mba: Keysym = Keysym(0x01000db9);
    /// Sinh_ya
    pub const Sinh_ya: Keysym = Keysym(0x01000dba);
    /// Sinh_ra
    pub const Sinh_ra: Keysym = Keysym(0x01000dbb);
    /// Sinh_la
    pub const Sinh_la: Keysym = Keysym(0x01000dbd);
    /// Sinh_va
    pub const Sinh_va: Keysym = Keysym(0x01000dc0);
    /// Sinh_sha
    pub const Sinh_sha: Keysym = Keysym(0x01000dc1);
    /// Sinh_ssha
    pub const Sinh_ssha: Keysym = Keysym(0x01000dc2);
    /// Sinh_sa
    pub const Sinh_sa: Keysym = Keysym(0x01000dc3);
    /// Sinh_ha
    pub const Sinh_ha: Keysym = Keysym(0x01000dc4);
    /// Sinh_lla
    pub const Sinh_lla: Keysym = Keysym(0x01000dc5);
    /// Sinh_fa
    pub const Sinh_fa: Keysym = Keysym(0x01000dc6);
    /// Sinh_al
    pub const Sinh_al: Keysym = Keysym(0x01000dca);
    /// Sinh_aa2
    pub const Sinh_aa2: Keysym = Keysym(0x01000dcf);
    /// Sinh_ae2
    pub const Sinh_ae2: Keysym = Keysym(0x01000dd0);
    /// Sinh_aee2
    pub const Sinh_aee2: Keysym = Keysym(0x01000dd1);
    /// Sinh_i2
    pub const Sinh_i2: Keysym = Keysym(0x01000dd2);
    /// Sinh_ii2
    pub const Sinh_ii2: Keysym = Keysym(0x01000dd3);
    /// Sinh_u2
    pub const Sinh_u2: Keysym = Keysym(0x01000dd4);
    /// Sinh_uu2
    pub const Sinh_uu2: Keysym = Keysym(0x01000dd6);
    /// Sinh_ru2
    pub const Sinh_ru2: Keysym = Keysym(0x01000dd8);
    /// Sinh_e2
    pub const Sinh_e2: Keysym = Keysym(0x01000dd9);
    /// Sinh_ee2
    pub const Sinh_ee2: Keysym = Keysym(0x01000dda);
    /// Sinh_ai2
    pub const Sinh_ai2: Keysym = Keysym(0x01000ddb);
    /// Sinh_o2
    pub const Sinh_o2: Keysym = Keysym(0x01000ddc);
    /// Sinh_oo2
    pub const Sinh_oo2: Keysym = Keysym(0x01000ddd);
    /// Sinh_au2
    pub const Sinh_au2: Keysym = Keysym(0x01000dde);
    /// Sinh_lu2
    pub const Sinh_lu2: Keysym = Keysym(0x01000ddf);
    /// Sinh_ruu2
    pub const Sinh_ruu2: Keysym = Keysym(0x01000df2);
    /// Sinh_luu2
    pub const Sinh_luu2: Keysym = Keysym(0x01000df3);
    /// Sinh_kunddaliya
    pub const Sinh_kunddaliya: Keysym = Keysym(0x01000df4);
    /// Georgian_an
    pub const Georgian_an: Keysym = Keysym(0x010010d0);
    /// Georgian_ban
    pub const Georgian_ban: Keysym = Keysym(0x010010d1);
    /// Georgian_gan
    pub const Georgian_gan: Keysym = Keysym(0x010010d2);
    /// Georgian_don
    pub const Georgian_don: Keysym = Keysym(0x010010d3);
    /// Georgian_en
    pub const Georgian_en: Keysym = Keysym(0x010010d4);
    /// Georgian_vin
    pub const Georgian_vin: Keysym = Keysym(0x010010d5);
    /// Georgian_zen
    pub const Georgian_zen: Keysym = Keysym(0x010010d6);
    /// Georgian_tan
    pub const Georgian_tan: Keysym = Keysym(0x010010d7);
    /// Georgian_in
    pub const Georgian_in: Keysym = Keysym(0x010010d8);
    /// Georgian_kan
    pub const Georgian_kan: Keysym = Keysym(0x010010d9);
    /// Georgian_las
    pub const Georgian_las: Keysym = Keysym(0x010010da);
    /// Georgian_man
    pub const Georgian_man: Keysym = Keysym(0x010010db);
    /// Georgian_nar
    pub const Georgian_nar: Keysym = Keysym(0x010010dc);
    /// Georgian_on
    pub const Georgian_on: Keysym = Keysym(0x010010dd);
    /// Georgian_par
    pub const Georgian_par: Keysym = Keysym(0x010010de);
    /// Georgian_zhar
    pub const Georgian_zhar: Keysym = Keysym(0x010010df);
    /// Georgian_rae
    pub const Georgian_rae: Keysym = Keysym(0x010010e0);
    /// Georgian_san
    pub const Georgian_san: Keysym = Keysym(0x010010e1);
    /// Georgian_tar
    pub const Georgian_tar: Keysym = Keysym(0x010010e2);
    /// Georgian_un
    pub const Georgian_un: Keysym = Keysym(0x010010e3);
    /// Georgian_phar
    pub const Georgian_phar: Keysym = Keysym(0x010010e4);
    /// Georgian_khar
    pub const Georgian_khar: Keysym = Keysym(0x010010e5);
    /// Georgian_ghan
    pub const Georgian_ghan: Keysym = Keysym(0x010010e6);
    /// Georgian_qar
    pub const Georgian_qar: Keysym = Keysym(0x010010e7);
    /// Georgian_shin
    pub const Georgian_shin: Keysym = Keysym(0x010010e8);
    /// Georgian_chin
    pub const Georgian_chin: Keysym = Keysym(0x010010e9);
    /// Georgian_can
    pub const Georgian_can: Keysym = Keysym(0x010010ea);
    /// Georgian_jil
    pub const Georgian_jil: Keysym = Keysym(0x010010eb);
    /// Georgian_cil
    pub const Georgian_cil: Keysym = Keysym(0x010010ec);
    /// Georgian_char
    pub const Georgian_char: Keysym = Keysym(0x010010ed);
    /// Georgian_xan
    pub const Georgian_xan: Keysym = Keysym(0x010010ee);
    /// Georgian_jhan
    pub const Georgian_jhan: Keysym = Keysym(0x010010ef);
    /// Georgian_hae
    pub const Georgian_hae: Keysym = Keysym(0x010010f0);
    /// Georgian_he
    pub const Georgian_he: Keysym = Keysym(0x010010f1);
    /// Georgian_hie
    pub const Georgian_hie: Keysym = Keysym(0x010010f2);
    /// Georgian_we
    pub const Georgian_we: Keysym = Keysym(0x010010f3);
    /// Georgian_har
    pub const Georgian_har: Keysym = Keysym(0x010010f4);
    /// Georgian_hoe
    pub const Georgian_hoe: Keysym = Keysym(0x010010f5);
    /// Georgian_fi
    pub const Georgian_fi: Keysym = Keysym(0x010010f6);
    /// Babovedot
    pub const Babovedot: Keysym = Keysym(0x01001e02);
    /// babovedot
    pub const babovedot: Keysym = Keysym(0x01001e03);
    /// Dabovedot
    pub const Dabovedot: Keysym = Keysym(0x01001e0a);
    /// dabovedot
    pub const dabovedot: Keysym = Keysym(0x01001e0b);
    /// Fabovedot
    pub const Fabovedot: Keysym = Keysym(0x01001e1e);
    /// fabovedot
    pub const fabovedot: Keysym = Keysym(0x01001e1f);
    /// Lbelowdot
    pub const Lbelowdot: Keysym = Keysym(0x01001e36);
    /// lbelowdot
    pub const lbelowdot: Keysym = Keysym(0x01001e37);
    /// Mabovedot
    pub const Mabovedot: Keysym = Keysym(0x01001e40);
    /// mabovedot
    pub const mabovedot: Keysym = Keysym(0x01001e41);
    /// Pabovedot
    pub const Pabovedot: Keysym = Keysym(0x01001e56);
    /// pabovedot
    pub const pabovedot: Keysym = Keysym(0x01001e57);
    /// Sabovedot
    pub const Sabovedot: Keysym = Keysym(0x01001e60);
    /// sabovedot
    pub const sabovedot: Keysym = Keysym(0x01001e61);
    /// Tabovedot
    pub const Tabovedot: Keysym = Keysym(0x01001e6a);
    /// tabovedot
    pub const tabovedot: Keysym = Keysym(0x01001e6b);
    /// Wgrave
    pub const Wgrave: Keysym = Keysym(0x01001e80);
    /// wgrave
    pub const wgrave: Keysym = Keysym(0x01001e81);
    /// Wacute
    pub const Wacute: Keysym = Keysym(0x01001e82);
    /// wacute
    pub const wacute: Keysym = Keysym(0x01001e83);
    /// Wdiaeresis
    pub const Wdiaeresis: Keysym = Keysym(0x01001e84);
    /// wdiaeresis
    pub const wdiaeresis: Keysym = Keysym(0x01001e85);
    /// Xabovedot
    pub const Xabovedot: Keysym = Keysym(0x01001e8a);
    /// xabovedot
    pub const xabovedot: Keysym = Keysym(0x01001e8b);
    /// Abelowdot
    pub const Abelowdot: Keysym = Keysym(0x01001ea0);
    /// abelowdot
    pub const abelowdot: Keysym = Keysym(0x01001ea1);
    /// Ahook
    pub const Ahook: Keysym = Keysym(0x01001ea2);
    /// ahook
    pub const ahook: Keysym = Keysym(0x01001ea3);
    /// Acircumflexacute
    pub const Acircumflexacute: Keysym = Keysym(0x01001ea4);
    /// acircumflexacute
    pub const acircumflexacute: Keysym = Keysym(0x01001ea5);
    /// Acircumflexgrave
    pub const Acircumflexgrave: Keysym = Keysym(0x01001ea6);
    /// acircumflexgrave
    pub const acircumflexgrave: Keysym = Keysym(0x01001ea7);
    /// Acircumflexhook
    pub const Acircumflexhook: Keysym = Keysym(0x01001ea8);
    /// acircumflexhook
    pub const acircumflexhook: Keysym = Keysym(0x01001ea9);
    /// Acircumflextilde
    pub const Acircumflextilde: Keysym = Keysym(0x01001eaa);
    /// acircumflextilde
    pub const acircumflextilde: Keysym = Keysym(0x01001eab);
    /// Acircumflexbelowdot
    pub const Acircumflexbelowdot: Keysym = Keysym(0x01001eac);
    /// acircumflexbelowdot
    pub const acircumflexbelowdot: Keysym = Keysym(0x01001ead);
    /// Abreveacute
    pub const Abreveacute: Keysym = Keysym(0x01001eae);
    /// abreveacute
    pub const abreveacute: Keysym = Keysym(0x01001eaf);
    /// Abrevegrave
    pub const Abrevegrave: Keysym = Keysym(0x01001eb0);
    /// abrevegrave
    pub const abrevegrave: Keysym = Keysym(0x01001eb1);
    /// Abrevehook
    pub const Abrevehook: Keysym = Keysym(0x01001eb2);
    /// abrevehook
    pub const abrevehook: Keysym = Keysym(0x01001eb3);
    /// Abrevetilde
    pub const Abrevetilde: Keysym = Keysym(0x01001eb4);
    /// abrevetilde
    pub const abrevetilde: Keysym = Keysym(0x01001eb5);
    /// Abrevebelowdot
    pub const Abrevebelowdot: Keysym = Keysym(0x01001eb6);
    /// abrevebelowdot
    pub const abrevebelowdot: Keysym = Keysym(0x01001eb7);
    /// Ebelowdot
    pub const Ebelowdot: Keysym = Keysym(0x01001eb8);
    /// ebelowdot
    pub const ebelowdot: Keysym = Keysym(0x01001eb9);
    /// Ehook
    pub const Ehook: Keysym = Keysym(0x01001eba);
    /// ehook
    pub const ehook: Keysym = Keysym(0x01001ebb);
    /// Etilde
    pub const Etilde: Keysym = Keysym(0x01001ebc);
    /// etilde
    pub const etilde: Keysym = Keysym(0x01001ebd);
    /// Ecircumflexacute
    pub const Ecircumflexacute: Keysym = Keysym(0x01001ebe);
    /// ecircumflexacute
    pub const ecircumflexacute: Keysym = Keysym(0x01001ebf);
    /// Ecircumflexgrave
    pub const Ecircumflexgrave: Keysym = Keysym(0x01001ec0);
    /// ecircumflexgrave
    pub const ecircumflexgrave: Keysym = Keysym(0x01001ec1);
    /// Ecircumflexhook
    pub const Ecircumflexhook: Keysym = Keysym(0x01001ec2);
    /// ecircumflexhook
    pub const ecircumflexhook: Keysym = Keysym(0x01001ec3);
    /// Ecircumflextilde
    pub const Ecircumflextilde: Keysym = Keysym(0x01001ec4);
    /// ecircumflextilde
    pub const ecircumflextilde: Keysym = Keysym(0x01001ec5);
    /// Ecircumflexbelowdot
    pub const Ecircumflexbelowdot: Keysym = Keysym(0x01001ec6);
    /// ecircumflexbelowdot
    pub const ecircumflexbelowdot: Keysym = Keysym(0x01001ec7);
    /// Ihook
    pub const Ihook: Keysym = Keysym(0x01001ec8);
    /// ihook
    pub const ihook: Keysym = Keysym(0x01001ec9);
    /// Ibelowdot
    pub const Ibelowdot: Keysym = Keysym(0x01001eca);
    /// ibelowdot
    pub const ibelowdot: Keysym = Keysym(0x01001ecb);
    /// Obelowdot
    pub const Obelowdot: Keysym = Keysym(0x01001ecc);
    /// obelowdot
    pub const obelowdot: Keysym = Keysym(0x01001ecd);
    /// Ohook
    pub const Ohook: Keysym = Keysym(0x01001ece);
    /// ohook
    pub const ohook: Keysym = Keysym(0x01001ecf);
    /// Ocircumflexacute
    pub const Ocircumflexacute: Keysym = Keysym(0x01001ed0);
    /// ocircumflexacute
    pub const ocircumflexacute: Keysym = Keysym(0x01001ed1);
    /// Ocircumflexgrave
    pub const Ocircumflexgrave: Keysym = Keysym(0x01001ed2);
    /// ocircumflexgrave
    pub const ocircumflexgrave: Keysym = Keysym(0x01001ed3);
    /// Ocircumflexhook
    pub const Ocircumflexhook: Keysym = Keysym(0x01001ed4);
    /// ocircumflexhook
    pub const ocircumflexhook: Keysym = Keysym(0x01001ed5);
    /// Ocircumflextilde
    pub const Ocircumflextilde: Keysym = Keysym(0x01001ed6);
    /// ocircumflextilde
    pub const ocircumflextilde: Keysym = Keysym(0x01001ed7);
    /// Ocircumflexbelowdot
    pub const Ocircumflexbelowdot: Keysym = Keysym(0x01001ed8);
    /// ocircumflexbelowdot
    pub const ocircumflexbelowdot: Keysym = Keysym(0x01001ed9);
    /// Ohornacute
    pub const Ohornacute: Keysym = Keysym(0x01001eda);
    /// ohornacute
    pub const ohornacute: Keysym = Keysym(0x01001edb);
    /// Ohorngrave
    pub const Ohorngrave: Keysym = Keysym(0x01001edc);
    /// ohorngrave
    pub const ohorngrave: Keysym = Keysym(0x01001edd);
    /// Ohornhook
    pub const Ohornhook: Keysym = Keysym(0x01001ede);
    /// ohornhook
    pub const ohornhook: Keysym = Keysym(0x01001edf);
    /// Ohorntilde
    pub const Ohorntilde: Keysym = Keysym(0x01001ee0);
    /// ohorntilde
    pub const ohorntilde: Keysym = Keysym(0x01001ee1);
    /// Ohornbelowdot
    pub const Ohornbelowdot: Keysym = Keysym(0x01001ee2);
    /// ohornbelowdot
    pub const ohornbelowdot: Keysym = Keysym(0x01001ee3);
    /// Ubelowdot
    pub const Ubelowdot: Keysym = Keysym(0x01001ee4);
    /// ubelowdot
    pub const ubelowdot: Keysym = Keysym(0x01001ee5);
    /// Uhook
    pub const Uhook: Keysym = Keysym(0x01001ee6);
    /// uhook
    pub const uhook: Keysym = Keysym(0x01001ee7);
    /// Uhornacute
    pub const Uhornacute: Keysym = Keysym(0x01001ee8);
    /// uhornacute
    pub const uhornacute: Keysym = Keysym(0x01001ee9);
    /// Uhorngrave
    pub const Uhorngrave: Keysym = Keysym(0x01001eea);
    /// uhorngrave
    pub const uhorngrave: Keysym = Keysym(0x01001eeb);
    /// Uhornhook
    pub const Uhornhook: Keysym = Keysym(0x01001eec);
    /// uhornhook
    pub const uhornhook: Keysym = Keysym(0x01001eed);
    /// Uhorntilde
    pub const Uhorntilde: Keysym = Keysym(0x01001eee);
    /// uhorntilde
    pub const uhorntilde: Keysym = Keysym(0x01001eef);
    /// Uhornbelowdot
    pub const Uhornbelowdot: Keysym = Keysym(0x01001ef0);
    /// uhornbelowdot
    pub const uhornbelowdot: Keysym = Keysym(0x01001ef1);
    /// Ygrave
    pub const Ygrave: Keysym = Keysym(0x01001ef2);
    /// ygrave
    pub const ygrave: Keysym = Keysym(0x01001ef3);
    /// Ybelowdot
    pub const Ybelowdot: Keysym = Keysym(0x01001ef4);
    /// ybelowdot
    pub const ybelowdot: Keysym = Keysym(0x01001ef5);
    /// Yhook
    pub const Yhook: Keysym = Keysym(0x01001ef6);
    /// yhook
    pub const yhook: Keysym = Keysym(0x01001ef7);
    /// Ytilde
    pub const Ytilde: Keysym = Keysym(0x01001ef8);
    /// ytilde
    pub const ytilde: Keysym = Keysym(0x01001ef9);
    /// zerosuperior
    pub const zerosuperior: Keysym = Keysym(0x01002070);
    /// foursuperior
    pub const foursuperior: Keysym = Keysym(0x01002074);
    /// fivesuperior
    pub const fivesuperior: Keysym = Keysym(0x01002075);
    /// sixsuperior
    pub const sixsuperior: Keysym = Keysym(0x01002076);
    /// sevensuperior
    pub const sevensuperior: Keysym = Keysym(0x01002077);
    /// eightsuperior
    pub const eightsuperior: Keysym = Keysym(0x01002078);
    /// ninesuperior
    pub const ninesuperior: Keysym = Keysym(0x01002079);
    /// zerosubscript
    pub const zerosubscript: Keysym = Keysym(0x01002080);
    /// onesubscript
    pub const onesubscript: Keysym = Keysym(0x01002081);
    /// twosubscript
    pub const twosubscript: Keysym = Keysym(0x01002082);
    /// threesubscript
    pub const threesubscript: Keysym = Keysym(0x01002083);
    /// foursubscript
    pub const foursubscript: Keysym = Keysym(0x01002084);
    /// fivesubscript
    pub const fivesubscript: Keysym = Keysym(0x01002085);
    /// sixsubscript
    pub const sixsubscript: Keysym = Keysym(0x01002086);
    /// sevensubscript
    pub const sevensubscript: Keysym = Keysym(0x01002087);
    /// eightsubscript
    pub const eightsubscript: Keysym = Keysym(0x01002088);
    /// ninesubscript
    pub const ninesubscript: Keysym = Keysym(0x01002089);
    /// EcuSign
    pub const EcuSign: Keysym = Keysym(0x010020a0);
    /// ColonSign
    pub const ColonSign: Keysym = Keysym(0x010020a1);
    /// CruzeiroSign
    pub const CruzeiroSign: Keysym = Keysym(0x010020a2);
    /// FFrancSign
    pub const FFrancSign: Keysym = Keysym(0x010020a3);
    /// LiraSign
    pub const LiraSign: Keysym = Keysym(0x010020a4);
    /// MillSign
    pub const MillSign: Keysym = Keysym(0x010020a5);
    /// NairaSign
    pub const NairaSign: Keysym = Keysym(0x010020a6);
    /// PesetaSign
    pub const PesetaSign: Keysym = Keysym(0x010020a7);
    /// RupeeSign
    pub const RupeeSign: Keysym = Keysym(0x010020a8);
    /// WonSign
    pub const WonSign: Keysym = Keysym(0x010020a9);
    /// NewSheqelSign
    pub const NewSheqelSign: Keysym = Keysym(0x010020aa);
    /// DongSign
    pub const DongSign: Keysym = Keysym(0x010020ab);
    /// partdifferential
    pub const partdifferential: Keysym = Keysym(0x01002202);
    /// emptyset
    pub const emptyset: Keysym = Keysym(0x01002205);
    /// elementof
    pub const elementof: Keysym = Keysym(0x01002208);
    /// notelementof
    pub const notelementof: Keysym = Keysym(0x01002209);
    /// containsas
    pub const containsas: Keysym = Keysym(0x0100220b);
    /// squareroot
    pub const squareroot: Keysym = Keysym(0x0100221a);
    /// cuberoot
    pub const cuberoot: Keysym = Keysym(0x0100221b);
    /// fourthroot
    pub const fourthroot: Keysym = Keysym(0x0100221c);
    /// dintegral
    pub const dintegral: Keysym = Keysym(0x0100222c);
    /// tintegral
    pub const tintegral: Keysym = Keysym(0x0100222d);
    /// because
    pub const because: Keysym = Keysym(0x01002235);
    /// notapproxeq
    pub const notapproxeq: Keysym = Keysym(0x01002247);
    /// approxeq
    pub const approxeq: Keysym = Keysym(0x01002248);
    /// notidentical
    pub const notidentical: Keysym = Keysym(0x01002262);
    /// stricteq
    pub const stricteq: Keysym = Keysym(0x01002263);
    /// braille_blank
    pub const braille_blank: Keysym = Keysym(0x01002800);
    /// braille_dots_1
    pub const braille_dots_1: Keysym = Keysym(0x01002801);
    /// braille_dots_2
    pub const braille_dots_2: Keysym = Keysym(0x01002802);
    /// braille_dots_12
    pub const braille_dots_12: Keysym = Keysym(0x01002803);
    /// braille_dots_3
    pub const braille_dots_3: Keysym = Keysym(0x01002804);
    /// braille_dots_13
    pub const braille_dots_13: Keysym = Keysym(0x01002805);
    /// braille_dots_23
    pub const braille_dots_23: Keysym = Keysym(0x01002806);
    /// braille_dots_123
    pub const braille_dots_123: Keysym = Keysym(0x01002807);
    /// braille_dots_4
    pub const braille_dots_4: Keysym = Keysym(0x01002808);
    /// braille_dots_14
    pub const braille_dots_14: Keysym = Keysym(0x01002809);
    /// braille_dots_24
    pub const braille_dots_24: Keysym = Keysym(0x0100280a);
    /// braille_dots_124
    pub const braille_dots_124: Keysym = Keysym(0x0100280b);
    /// braille_dots_34
    pub const braille_dots_34: Keysym = Keysym(0x0100280c);
    /// braille_dots_134
    pub const braille_dots_134: Keysym = Keysym(0x0100280d);
    /// braille_dots_234
    pub const braille_dots_234: Keysym = Keysym(0x0100280e);
    /// braille_dots_1234
    pub const braille_dots_1234: Keysym = Keysym(0x0100280f);
    /// braille_dots_5
    pub const braille_dots_5: Keysym = Keysym(0x01002810);
    /// braille_dots_15
    pub const braille_dots_15: Keysym = Keysym(0x01002811);
    /// braille_dots_25
    pub const braille_dots_25: Keysym = Keysym(0x01002812);
    /// braille_dots_125
    pub const braille_dots_125: Keysym = Keysym(0x01002813);
    /// braille_dots_35
    pub const braille_dots_35: Keysym = Keysym(0x01002814);
    /// braille_dots_135
    pub const braille_dots_135: Keysym = Keysym(0x01002815);
    /// braille_dots_235
    pub const braille_dots_235: Keysym = Keysym(0x01002816);
    /// braille_dots_1235
    pub const braille_dots_1235: Keysym = Keysym(0x01002817);
    /// braille_dots_45
    pub const braille_dots_45: Keysym = Keysym(0x01002818);
    /// braille_dots_145
    pub const braille_dots_145: Keysym = Keysym(0x01002819);
    /// braille_dots_245
    pub const braille_dots_245: Keysym = Keysym(0x0100281a);
    /// braille_dots_1245
    pub const braille_dots_1245: Keysym = Keysym(0x0100281b);
    /// braille_dots_345
    pub const braille_dots_345: Keysym = Keysym(0x0100281c);
    /// braille_dots_1345
    pub const braille_dots_1345: Keysym = Keysym(0x0100281d);
    /// braille_dots_2345
    pub const braille_dots_2345: Keysym = Keysym(0x0100281e);
    /// braille_dots_12345
    pub const braille_dots_12345: Keysym = Keysym(0x0100281f);
    /// braille_dots_6
    pub const braille_dots_6: Keysym = Keysym(0x01002820);
    /// braille_dots_16
    pub const braille_dots_16: Keysym = Keysym(0x01002821);
    /// braille_dots_26
    pub const braille_dots_26: Keysym = Keysym(0x01002822);
    /// braille_dots_126
    pub const braille_dots_126: Keysym = Keysym(0x01002823);
    /// braille_dots_36
    pub const braille_dots_36: Keysym = Keysym(0x01002824);
    /// braille_dots_136
    pub const braille_dots_136: Keysym = Keysym(0x01002825);
    /// braille_dots_236
    pub const braille_dots_236: Keysym = Keysym(0x01002826);
    /// braille_dots_1236
    pub const braille_dots_1236: Keysym = Keysym(0x01002827);
    /// braille_dots_46
    pub const braille_dots_46: Keysym = Keysym(0x01002828);
    /// braille_dots_146
    pub const braille_dots_146: Keysym = Keysym(0x01002829);
    /// braille_dots_246
    pub const braille_dots_246: Keysym = Keysym(0x0100282a);
    /// braille_dots_1246
    pub const braille_dots_1246: Keysym = Keysym(0x0100282b);
    /// braille_dots_346
    pub const braille_dots_346: Keysym = Keysym(0x0100282c);
    /// braille_dots_1346
    pub const braille_dots_1346: Keysym = Keysym(0x0100282d);
    /// braille_dots_2346
    pub const braille_dots_2346: Keysym = Keysym(0x0100282e);
    /// braille_dots_12346
    pub const braille_dots_12346: Keysym = Keysym(0x0100282f);
    /// braille_dots_56
    pub const braille_dots_56: Keysym = Keysym(0x01002830);
    /// braille_dots_156
    pub const braille_dots_156: Keysym = Keysym(0x01002831);
    /// braille_dots_256
    pub const braille_dots_256: Keysym = Keysym(0x01002832);
    /// braille_dots_1256
    pub const braille_dots_1256: Keysym = Keysym(0x01002833);
    /// braille_dots_356
    pub const braille_dots_356: Keysym = Keysym(0x01002834);
    /// braille_dots_1356
    pub const braille_dots_1356: Keysym = Keysym(0x01002835);
    /// braille_dots_2356
    pub const braille_dots_2356: Keysym = Keysym(0x01002836);
    /// braille_dots_12356
    pub const braille_dots_12356: Keysym = Keysym(0x01002837);
    /// braille_dots_456
    pub const braille_dots_456: Keysym = Keysym(0x01002838);
    /// braille_dots_1456
    pub const braille_dots_1456: Keysym = Keysym(0x01002839);
    /// braille_dots_2456
    pub const braille_dots_2456: Keysym = Keysym(0x0100283a);
    /// braille_dots_12456
    pub const braille_dots_12456: Keysym = Keysym(0x0100283b);
    /// braille_dots_3456
    pub const braille_dots_3456: Keysym = Keysym(0x0100283c);
    /// braille_dots_13456
    pub const braille_dots_13456: Keysym = Keysym(0x0100283d);
    /// braille_dots_23456
    pub const braille_dots_23456: Keysym = Keysym(0x0100283e);
    /// braille_dots_123456
    pub const braille_dots_123456: Keysym = Keysym(0x0100283f);
    /// braille_dots_7
    pub const braille_dots_7: Keysym = Keysym(0x01002840);
    /// braille_dots_17
    pub const braille_dots_17: Keysym = Keysym(0x01002841);
    /// braille_dots_27
    pub const braille_dots_27: Keysym = Keysym(0x01002842);
    /// braille_dots_127
    pub const braille_dots_127: Keysym = Keysym(0x01002843);
    /// braille_dots_37
    pub const braille_dots_37: Keysym = Keysym(0x01002844);
    /// braille_dots_137
    pub const braille_dots_137: Keysym = Keysym(0x01002845);
    /// braille_dots_237
    pub const braille_dots_237: Keysym = Keysym(0x01002846);
    /// braille_dots_1237
    pub const braille_dots_1237: Keysym = Keysym(0x01002847);
    /// braille_dots_47
    pub const braille_dots_47: Keysym = Keysym(0x01002848);
    /// braille_dots_147
    pub const braille_dots_147: Keysym = Keysym(0x01002849);
    /// braille_dots_247
    pub const braille_dots_247: Keysym = Keysym(0x0100284a);
    /// braille_dots_1247
    pub const braille_dots_1247: Keysym = Keysym(0x0100284b);
    /// braille_dots_347
    pub const braille_dots_347: Keysym = Keysym(0x0100284c);
    /// braille_dots_1347
    pub const braille_dots_1347: Keysym = Keysym(0x0100284d);
    /// braille_dots_2347
    pub const braille_dots_2347: Keysym = Keysym(0x0100284e);
    /// braille_dots_12347
    pub const braille_dots_12347: Keysym = Keysym(0x0100284f);
    /// braille_dots_57
    pub const braille_dots_57: Keysym = Keysym(0x01002850);
    /// braille_dots_157
    pub const braille_dots_157: Keysym = Keysym(0x01002851);
    /// braille_dots_257
    pub const braille_dots_257: Keysym = Keysym(0x01002852);
    /// braille_dots_1257
    pub const braille_dots_1257: Keysym = Keysym(0x01002853);
    /// braille_dots_357
    pub const braille_dots_357: Keysym = Keysym(0x01002854);
    /// braille_dots_1357
    pub const braille_dots_1357: Keysym = Keysym(0x01002855);
    /// braille_dots_2357
    pub const braille_dots_2357: Keysym = Keysym(0x01002856);
    /// braille_dots_12357
    pub const braille_dots_12357: Keysym = Keysym(0x01002857);
    /// braille_dots_457
    pub const braille_dots_457: Keysym = Keysym(0x01002858);
    /// braille_dots_1457
    pub const braille_dots_1457: Keysym = Keysym(0x01002859);
    /// braille_dots_2457
    pub const braille_dots_2457: Keysym = Keysym(0x0100285a);
    /// braille_dots_12457
    pub const braille_dots_12457: Keysym = Keysym(0x0100285b);
    /// braille_dots_3457
    pub const braille_dots_3457: Keysym = Keysym(0x0100285c);
    /// braille_dots_13457
    pub const braille_dots_13457: Keysym = Keysym(0x0100285d);
    /// braille_dots_23457
    pub const braille_dots_23457: Keysym = Keysym(0x0100285e);
    /// braille_dots_123457
    pub const braille_dots_123457: Keysym = Keysym(0x0100285f);
    /// braille_dots_67
    pub const braille_dots_67: Keysym = Keysym(0x01002860);
    /// braille_dots_167
    pub const braille_dots_167: Keysym = Keysym(0x01002861);
    /// braille_dots_267
    pub const braille_dots_267: Keysym = Keysym(0x01002862);
    /// braille_dots_1267
    pub const braille_dots_1267: Keysym = Keysym(0x01002863);
    /// braille_dots_367
    pub const braille_dots_367: Keysym = Keysym(0x01002864);
    /// braille_dots_1367
    pub const braille_dots_1367: Keysym = Keysym(0x01002865);
    /// braille_dots_2367
    pub const braille_dots_2367: Keysym = Keysym(0x01002866);
    /// braille_dots_12367
    pub const braille_dots_12367: Keysym = Keysym(0x01002867);
    /// braille_dots_467
    pub const braille_dots_467: Keysym = Keysym(0x01002868);
    /// braille_dots_1467
    pub const braille_dots_1467: Keysym = Keysym(0x01002869);
    /// braille_dots_2467
    pub const braille_dots_2467: Keysym = Keysym(0x0100286a);
    /// braille_dots_12467
    pub const braille_dots_12467: Keysym = Keysym(0x0100286b);
    /// braille_dots_3467
    pub const braille_dots_3467: Keysym = Keysym(0x0100286c);
    /// braille_dots_13467
    pub const braille_dots_13467: Keysym = Keysym(0x0100286d);
    /// braille_dots_23467
    pub const braille_dots_23467: Keysym = Keysym(0x0100286e);
    /// braille_dots_123467
    pub const braille_dots_123467: Keysym = Keysym(0x0100286f);
    /// braille_dots_567
    pub const braille_dots_567: Keysym = Keysym(0x01002870);
    /// braille_dots_1567
    pub const braille_dots_1567: Keysym = Keysym(0x01002871);
    /// braille_dots_2567
    pub const braille_dots_2567: Keysym = Keysym(0x01002872);
    /// braille_dots_12567
    pub const braille_dots_12567: Keysym = Keysym(0x01002873);
    /// braille_dots_3567
    pub const braille_dots_3567: Keysym = Keysym(0x01002874);
    /// braille_dots_13567
    pub const braille_dots_13567: Keysym = Keysym(0x01002875);
    /// braille_dots_23567
    pub const braille_dots_23567: Keysym = Keysym(0x01002876);
    /// braille_dots_123567
    pub const braille_dots_123567: Keysym = Keysym(0x01002877);
    /// braille_dots_4567
    pub const braille_dots_4567: Keysym = Keysym(0x01002878);
    /// braille_dots_14567
    pub const braille_dots_14567: Keysym = Keysym(0x01002879);
    /// braille_dots_24567
    pub const braille_dots_24567: Keysym = Keysym(0x0100287a);
    /// braille_dots_124567
    pub const braille_dots_124567: Keysym = Keysym(0x0100287b);
    /// braille_dots_34567
    pub const braille_dots_34567: Keysym = Keysym(0x0100287c);
    /// braille_dots_134567
    pub const braille_dots_134567: Keysym = Keysym(0x0100287d);
    /// braille_dots_234567
    pub const braille_dots_234567: Keysym = Keysym(0x0100287e);
    /// braille_dots_1234567
    pub const braille_dots_1234567: Keysym = Keysym(0x0100287f);
    /// braille_dots_8
    pub const braille_dots_8: Keysym = Keysym(0x01002880);
    /// braille_dots_18
    pub const braille_dots_18: Keysym = Keysym(0x01002881);
    /// braille_dots_28
    pub const braille_dots_28: Keysym = Keysym(0x01002882);
    /// braille_dots_128
    pub const braille_dots_128: Keysym = Keysym(0x01002883);
    /// braille_dots_38
    pub const braille_dots_38: Keysym = Keysym(0x01002884);
    /// braille_dots_138
    pub const braille_dots_138: Keysym = Keysym(0x01002885);
    /// braille_dots_238
    pub const braille_dots_238: Keysym = Keysym(0x01002886);
    /// braille_dots_1238
    pub const braille_dots_1238: Keysym = Keysym(0x01002887);
    /// braille_dots_48
    pub const braille_dots_48: Keysym = Keysym(0x01002888);
    /// braille_dots_148
    pub const braille_dots_148: Keysym = Keysym(0x01002889);
    /// braille_dots_248
    pub const braille_dots_248: Keysym = Keysym(0x0100288a);
    /// braille_dots_1248
    pub const braille_dots_1248: Keysym = Keysym(0x0100288b);
    /// braille_dots_348
    pub const braille_dots_348: Keysym = Keysym(0x0100288c);
    /// braille_dots_1348
    pub const braille_dots_1348: Keysym = Keysym(0x0100288d);
    /// braille_dots_2348
    pub const braille_dots_2348: Keysym = Keysym(0x0100288e);
    /// braille_dots_12348
    pub const braille_dots_12348: Keysym = Keysym(0x0100288f);
    /// braille_dots_58
    pub const braille_dots_58: Keysym = Keysym(0x01002890);
    /// braille_dots_158
    pub const braille_dots_158: Keysym = Keysym(0x01002891);
    /// braille_dots_258
    pub const braille_dots_258: Keysym = Keysym(0x01002892);
    /// braille_dots_1258
    pub const braille_dots_1258: Keysym = Keysym(0x01002893);
    /// braille_dots_358
    pub const braille_dots_358: Keysym = Keysym(0x01002894);
    /// braille_dots_1358
    pub const braille_dots_1358: Keysym = Keysym(0x01002895);
    /// braille_dots_2358
    pub const braille_dots_2358: Keysym = Keysym(0x01002896);
    /// braille_dots_12358
    pub const braille_dots_12358: Keysym = Keysym(0x01002897);
    /// braille_dots_458
    pub const braille_dots_458: Keysym = Keysym(0x01002898);
    /// braille_dots_1458
    pub const braille_dots_1458: Keysym = Keysym(0x01002899);
    /// braille_dots_2458
    pub const braille_dots_2458: Keysym = Keysym(0x0100289a);
    /// braille_dots_12458
    pub const braille_dots_12458: Keysym = Keysym(0x0100289b);
    /// braille_dots_3458
    pub const braille_dots_3458: Keysym = Keysym(0x0100289c);
    /// braille_dots_13458
    pub const braille_dots_13458: Keysym = Keysym(0x0100289d);
    /// braille_dots_23458
    pub const braille_dots_23458: Keysym = Keysym(0x0100289e);
    /// braille_dots_123458
    pub const braille_dots_123458: Keysym = Keysym(0x0100289f);
    /// braille_dots_68
    pub const braille_dots_68: Keysym = Keysym(0x010028a0);
    /// braille_dots_168
    pub const braille_dots_168: Keysym = Keysym(0x010028a1);
    /// braille_dots_268
    pub const braille_dots_268: Keysym = Keysym(0x010028a2);
    /// braille_dots_1268
    pub const braille_dots_1268: Keysym = Keysym(0x010028a3);
    /// braille_dots_368
    pub const braille_dots_368: Keysym = Keysym(0x010028a4);
    /// braille_dots_1368
    pub const braille_dots_1368: Keysym = Keysym(0x010028a5);
    /// braille_dots_2368
    pub const braille_dots_2368: Keysym = Keysym(0x010028a6);
    /// braille_dots_12368
    pub const braille_dots_12368: Keysym = Keysym(0x010028a7);
    /// braille_dots_468
    pub const braille_dots_468: Keysym = Keysym(0x010028a8);
    /// braille_dots_1468
    pub const braille_dots_1468: Keysym = Keysym(0x010028a9);
    /// braille_dots_2468
    pub const braille_dots_2468: Keysym = Keysym(0x010028aa);
    /// braille_dots_12468
    pub const braille_dots_12468: Keysym = Keysym(0x010028ab);
    /// braille_dots_3468
    pub const braille_dots_3468: Keysym = Keysym(0x010028ac);
    /// braille_dots_13468
    pub const braille_dots_13468: Keysym = Keysym(0x010028ad);
    /// braille_dots_23468
    pub const braille_dots_23468: Keysym = Keysym(0x010028ae);
    /// braille_dots_123468
    pub const braille_dots_123468: Keysym = Keysym(0x010028af);
    /// braille_dots_568
    pub const braille_dots_568: Keysym = Keysym(0x010028b0);
    /// braille_dots_1568
    pub const braille_dots_1568: Keysym = Keysym(0x010028b1);
    /// braille_dots_2568
    pub const braille_dots_2568: Keysym = Keysym(0x010028b2);
    /// braille_dots_12568
    pub const braille_dots_12568: Keysym = Keysym(0x010028b3);
    /// braille_dots_3568
    pub const braille_dots_3568: Keysym = Keysym(0x010028b4);
    /// braille_dots_13568
    pub const braille_dots_13568: Keysym = Keysym(0x010028b5);
    /// braille_dots_23568
    pub const braille_dots_23568: Keysym = Keysym(0x010028b6);
    /// braille_dots_123568
    pub const braille_dots_123568: Keysym = Keysym(0x010028b7);
    /// braille_dots_4568
    pub const braille_dots_4568: Keysym = Keysym(0x010028b8);
    /// braille_dots_14568
    pub const braille_dots_14568: Keysym = Keysym(0x010028b9);
    /// braille_dots_24568
    pub const braille_dots_24568: Keysym = Keysym(0x010028ba);
    /// braille_dots_124568
    pub const braille_dots_124568: Keysym = Keysym(0x010028bb);
    /// braille_dots_34568
    pub const braille_dots_34568: Keysym = Keysym(0x010028bc);
    /// braille_dots_134568
    pub const braille_dots_134568: Keysym = Keysym(0x010028bd);
    /// braille_dots_234568
    pub const braille_dots_234568: Keysym = Keysym(0x010028be);
    /// braille_dots_1234568
    pub const braille_dots_1234568: Keysym = Keysym(0x010028bf);
    /// braille_dots_78
    pub const braille_dots_78: Keysym = Keysym(0x010028c0);
    /// braille_dots_178
    pub const braille_dots_178: Keysym = Keysym(0x010028c1);
    /// braille_dots_278
    pub const braille_dots_278: Keysym = Keysym(0x010028c2);
    /// braille_dots_1278
    pub const braille_dots_1278: Keysym = Keysym(0x010028c3);
    /// braille_dots_378
    pub const braille_dots_378: Keysym = Keysym(0x010028c4);
    /// braille_dots_1378
    pub const braille_dots_1378: Keysym = Keysym(0x010028c5);
    /// braille_dots_2378
    pub const braille_dots_2378: Keysym = Keysym(0x010028c6);
    /// braille_dots_12378
    pub const braille_dots_12378: Keysym = Keysym(0x010028c7);
    /// braille_dots_478
    pub const braille_dots_478: Keysym = Keysym(0x010028c8);
    /// braille_dots_1478
    pub const braille_dots_1478: Keysym = Keysym(0x010028c9);
    /// braille_dots_2478
    pub const braille_dots_2478: Keysym = Keysym(0x010028ca);
    /// braille_dots_12478
    pub const braille_dots_12478: Keysym = Keysym(0x010028cb);
    /// braille_dots_3478
    pub const braille_dots_3478: Keysym = Keysym(0x010028cc);
    /// braille_dots_13478
    pub const braille_dots_13478: Keysym = Keysym(0x010028cd);
    /// braille_dots_23478
    pub const braille_dots_23478: Keysym = Keysym(0x010028ce);
    /// braille_dots_123478
    pub const braille_dots_123478: Keysym = Keysym(0x010028cf);
    /// braille_dots_578
    pub const braille_dots_578: Keysym = Keysym(0x010028d0);
    /// braille_dots_1578
    pub const braille_dots_1578: Keysym = Keysym(0x010028d1);
    /// braille_dots_2578
    pub const braille_dots_2578: Keysym = Keysym(0x010028d2);
    /// braille_dots_12578
    pub const braille_dots_12578: Keysym = Keysym(0x010028d3);
    /// braille_dots_3578
    pub const braille_dots_3578: Keysym = Keysym(0x010028d4);
    /// braille_dots_13578
    pub const braille_dots_13578: Keysym = Keysym(0x010028d5);
    /// braille_dots_23578
    pub const braille_dots_23578: Keysym = Keysym(0x010028d6);
    /// braille_dots_123578
    pub const braille_dots_123578: Keysym = Keysym(0x010028d7);
    /// braille_dots_4578
    pub const braille_dots_4578: Keysym = Keysym(0x010028d8);
    /// braille_dots_14578
    pub const braille_dots_14578: Keysym = Keysym(0x010028d9);
    /// braille_dots_24578
    pub const braille_dots_24578: Keysym = Keysym(0x010028da);
    /// braille_dots_124578
    pub const braille_dots_124578: Keysym = Keysym(0x010028db);
    /// braille_dots_34578
    pub const braille_dots_34578: Keysym = Keysym(0x010028dc);
    /// braille_dots_134578
    pub const braille_dots_134578: Keysym = Keysym(0x010028dd);
    /// braille_dots_234578
    pub const braille_dots_234578: Keysym = Keysym(0x010028de);
    /// braille_dots_1234578
    pub const braille_dots_1234578: Keysym = Keysym(0x010028df);
    /// braille_dots_678
    pub const braille_dots_678: Keysym = Keysym(0x010028e0);
    /// braille_dots_1678
    pub const braille_dots_1678: Keysym = Keysym(0x010028e1);
    /// braille_dots_2678
    pub const braille_dots_2678: Keysym = Keysym(0x010028e2);
    /// braille_dots_12678
    pub const braille_dots_12678: Keysym = Keysym(0x010028e3);
    /// braille_dots_3678
    pub const braille_dots_3678: Keysym = Keysym(0x010028e4);
    /// braille_dots_13678
    pub const braille_dots_13678: Keysym = Keysym(0x010028e5);
    /// braille_dots_23678
    pub const braille_dots_23678: Keysym = Keysym(0x010028e6);
    /// braille_dots_123678
    pub const braille_dots_123678: Keysym = Keysym(0x010028e7);
    /// braille_dots_4678
    pub const braille_dots_4678: Keysym = Keysym(0x010028e8);
    /// braille_dots_14678
    pub const braille_dots_14678: Keysym = Keysym(0x010028e9);
    /// braille_dots_24678
    pub const braille_dots_24678: Keysym = Keysym(0x010028ea);
    /// braille_dots_124678
    pub const braille_dots_124678: Keysym = Keysym(0x010028eb);
    /// braille_dots_34678
    pub const braille_dots_34678: Keysym = Keysym(0x010028ec);
    /// braille_dots_134678
    pub const braille_dots_134678: Keysym = Keysym(0x010028ed);
    /// braille_dots_234678
    pub const braille_dots_234678: Keysym = Keysym(0x010028ee);
    /// braille_dots_1234678
    pub const braille_dots_1234678: Keysym = Keysym(0x010028ef);
    /// braille_dots_5678
    pub const braille_dots_5678: Keysym = Keysym(0x010028f0);
    /// braille_dots_15678
    pub const braille_dots_15678: Keysym = Keysym(0x010028f1);
    /// braille_dots_25678
    pub const braille_dots_25678: Keysym = Keysym(0x010028f2);
    /// braille_dots_125678
    pub const braille_dots_125678: Keysym = Keysym(0x010028f3);
    /// braille_dots_35678
    pub const braille_dots_35678: Keysym = Keysym(0x010028f4);
    /// braille_dots_135678
    pub const braille_dots_135678: Keysym = Keysym(0x010028f5);
    /// braille_dots_235678
    pub const braille_dots_235678: Keysym = Keysym(0x010028f6);
    /// braille_dots_1235678
    pub const braille_dots_1235678: Keysym = Keysym(0x010028f7);
    /// braille_dots_45678
    pub const braille_dots_45678: Keysym = Keysym(0x010028f8);
    /// braille_dots_145678
    pub const braille_dots_145678: Keysym = Keysym(0x010028f9);
    /// braille_dots_245678
    pub const braille_dots_245678: Keysym = Keysym(0x010028fa);
    /// braille_dots_1245678
    pub const braille_dots_1245678: Keysym = Keysym(0x010028fb);
    /// braille_dots_345678
    pub const braille_dots_345678: Keysym = Keysym(0x010028fc);
    /// braille_dots_1345678
    pub const braille_dots_1345678: Keysym = Keysym(0x010028fd);
    /// braille_dots_2345678
    pub const braille_dots_2345678: Keysym = Keysym(0x010028fe);
    /// braille_dots_12345678
    pub const braille_dots_12345678: Keysym = Keysym(0x010028ff);
    /// hpmute_acute
    pub const hpmute_acute: Keysym = Keysym(0x100000a8);
    /// mute_acute
    pub const mute_acute: Keysym = Keysym(0x100000a8);
    /// hpmute_grave
    pub const hpmute_grave: Keysym = Keysym(0x100000a9);
    /// mute_grave
    pub const mute_grave: Keysym = Keysym(0x100000a9);
    /// hpmute_asciicircum
    pub const hpmute_asciicircum: Keysym = Keysym(0x100000aa);
    /// mute_asciicircum
    pub const mute_asciicircum: Keysym = Keysym(0x100000aa);
    /// hpmute_diaeresis
    pub const hpmute_diaeresis: Keysym = Keysym(0x100000ab);
    /// mute_diaeresis
    pub const mute_diaeresis: Keysym = Keysym(0x100000ab);
    /// hpmute_asciitilde
    pub const hpmute_asciitilde: Keysym = Keysym(0x100000ac);
    /// mute_asciitilde
    pub const mute_asciitilde: Keysym = Keysym(0x100000ac);
    /// hplira
    pub const hplira: Keysym = Keysym(0x100000af);
    /// lira
    pub const lira: Keysym = Keysym(0x100000af);
    /// hpguilder
    pub const hpguilder: Keysym = Keysym(0x100000be);
    /// guilder
    pub const guilder: Keysym = Keysym(0x100000be);
    /// hpYdiaeresis
    pub const hpYdiaeresis: Keysym = Keysym(0x100000ee);
    /// hpIO
    pub const hpIO: Keysym = Keysym(0x100000ee);
    /// IO
    pub const IO: Keysym = Keysym(0x100000ee);
    /// hplongminus
    pub const hplongminus: Keysym = Keysym(0x100000f6);
    /// longminus
    pub const longminus: Keysym = Keysym(0x100000f6);
    /// hpblock
    pub const hpblock: Keysym = Keysym(0x100000fc);
    /// block
    pub const block: Keysym = Keysym(0x100000fc);
    /// Ddiaeresis
    pub const Ddiaeresis: Keysym = Keysym(0x1000fe22);
    /// Dacute_accent
    pub const Dacute_accent: Keysym = Keysym(0x1000fe27);
    /// Dcedilla_accent
    pub const Dcedilla_accent: Keysym = Keysym(0x1000fe2c);
    /// Dcircumflex_accent
    pub const Dcircumflex_accent: Keysym = Keysym(0x1000fe5e);
    /// Dgrave_accent
    pub const Dgrave_accent: Keysym = Keysym(0x1000fe60);
    /// Dtilde
    pub const Dtilde: Keysym = Keysym(0x1000fe7e);
    /// Dring_accent
    pub const Dring_accent: Keysym = Keysym(0x1000feb0);
    /// DRemove
    pub const DRemove: Keysym = Keysym(0x1000ff00);
    /// hpModelock1
    pub const hpModelock1: Keysym = Keysym(0x1000ff48);
    /// hpModelock2
    pub const hpModelock2: Keysym = Keysym(0x1000ff49);
    /// hpReset
    pub const hpReset: Keysym = Keysym(0x1000ff6c);
    /// Reset
    pub const Reset: Keysym = Keysym(0x1000ff6c);
    /// hpSystem
    pub const hpSystem: Keysym = Keysym(0x1000ff6d);
    /// System
    pub const System: Keysym = Keysym(0x1000ff6d);
    /// hpUser
    pub const hpUser: Keysym = Keysym(0x1000ff6e);
    /// User
    pub const User: Keysym = Keysym(0x1000ff6e);
    /// hpClearLine
    pub const hpClearLine: Keysym = Keysym(0x1000ff6f);
    /// ClearLine
    pub const ClearLine: Keysym = Keysym(0x1000ff6f);
    /// hpInsertLine
    pub const hpInsertLine: Keysym = Keysym(0x1000ff70);
    /// InsertLine
    pub const InsertLine: Keysym = Keysym(0x1000ff70);
    /// hpDeleteLine
    pub const hpDeleteLine: Keysym = Keysym(0x1000ff71);
    /// DeleteLine
    pub const DeleteLine: Keysym = Keysym(0x1000ff71);
    /// hpInsertChar
    pub const hpInsertChar: Keysym = Keysym(0x1000ff72);
    /// InsertChar
    pub const InsertChar: Keysym = Keysym(0x1000ff72);
    /// hpDeleteChar
    pub const hpDeleteChar: Keysym = Keysym(0x1000ff73);
    /// DeleteChar
    pub const DeleteChar: Keysym = Keysym(0x1000ff73);
    /// hpBackTab
    pub const hpBackTab: Keysym = Keysym(0x1000ff74);
    /// BackTab
    pub const BackTab: Keysym = Keysym(0x1000ff74);
    /// hpKP_BackTab
    pub const hpKP_BackTab: Keysym = Keysym(0x1000ff75);
    /// KP_BackTab
    pub const KP_BackTab: Keysym = Keysym(0x1000ff75);
    /// Ext16bit_L
    pub const Ext16bit_L: Keysym = Keysym(0x1000ff76);
    /// Ext16bit_R
    pub const Ext16bit_R: Keysym = Keysym(0x1000ff77);
    /// osfCopy
    pub const osfCopy: Keysym = Keysym(0x1004ff02);
    /// osfCut
    pub const osfCut: Keysym = Keysym(0x1004ff03);
    /// osfPaste
    pub const osfPaste: Keysym = Keysym(0x1004ff04);
    /// osfBackTab
    pub const osfBackTab: Keysym = Keysym(0x1004ff07);
    /// osfBackSpace
    pub const osfBackSpace: Keysym = Keysym(0x1004ff08);
    /// osfClear
    pub const osfClear: Keysym = Keysym(0x1004ff0b);
    /// osfEscape
    pub const osfEscape: Keysym = Keysym(0x1004ff1b);
    /// osfAddMode
    pub const osfAddMode: Keysym = Keysym(0x1004ff31);
    /// osfPrimaryPaste
    pub const osfPrimaryPaste: Keysym = Keysym(0x1004ff32);
    /// osfQuickPaste
    pub const osfQuickPaste: Keysym = Keysym(0x1004ff33);
    /// osfPageLeft
    pub const osfPageLeft: Keysym = Keysym(0x1004ff40);
    /// osfPageUp
    pub const osfPageUp: Keysym = Keysym(0x1004ff41);
    /// osfPageDown
    pub const osfPageDown: Keysym = Keysym(0x1004ff42);
    /// osfPageRight
    pub const osfPageRight: Keysym = Keysym(0x1004ff43);
    /// osfActivate
    pub const osfActivate: Keysym = Keysym(0x1004ff44);
    /// osfMenuBar
    pub const osfMenuBar: Keysym = Keysym(0x1004ff45);
    /// osfLeft
    pub const osfLeft: Keysym = Keysym(0x1004ff51);
    /// osfUp
    pub const osfUp: Keysym = Keysym(0x1004ff52);
    /// osfRight
    pub const osfRight: Keysym = Keysym(0x1004ff53);
    /// osfDown
    pub const osfDown: Keysym = Keysym(0x1004ff54);
    /// osfEndLine
    pub const osfEndLine: Keysym = Keysym(0x1004ff57);
    /// osfBeginLine
    pub const osfBeginLine: Keysym = Keysym(0x1004ff58);
    /// osfEndData
    pub const osfEndData: Keysym = Keysym(0x1004ff59);
    /// osfBeginData
    pub const osfBeginData: Keysym = Keysym(0x1004ff5a);
    /// osfPrevMenu
    pub const osfPrevMenu: Keysym = Keysym(0x1004ff5b);
    /// osfNextMenu
    pub const osfNextMenu: Keysym = Keysym(0x1004ff5c);
    /// osfPrevField
    pub const osfPrevField: Keysym = Keysym(0x1004ff5d);
    /// osfNextField
    pub const osfNextField: Keysym = Keysym(0x1004ff5e);
    /// osfSelect
    pub const osfSelect: Keysym = Keysym(0x1004ff60);
    /// osfInsert
    pub const osfInsert: Keysym = Keysym(0x1004ff63);
    /// osfUndo
    pub const osfUndo: Keysym = Keysym(0x1004ff65);
    /// osfMenu
    pub const osfMenu: Keysym = Keysym(0x1004ff67);
    /// osfCancel
    pub const osfCancel: Keysym = Keysym(0x1004ff69);
    /// osfHelp
    pub const osfHelp: Keysym = Keysym(0x1004ff6a);
    /// osfSelectAll
    pub const osfSelectAll: Keysym = Keysym(0x1004ff71);
    /// osfDeselectAll
    pub const osfDeselectAll: Keysym = Keysym(0x1004ff72);
    /// osfReselect
    pub const osfReselect: Keysym = Keysym(0x1004ff73);
    /// osfExtend
    pub const osfExtend: Keysym = Keysym(0x1004ff74);
    /// osfRestore
    pub const osfRestore: Keysym = Keysym(0x1004ff78);
    /// osfDelete
    pub const osfDelete: Keysym = Keysym(0x1004ffff);
    /// SunFA_Grave
    pub const SunFA_Grave: Keysym = Keysym(0x1005ff00);
    /// SunFA_Circum
    pub const SunFA_Circum: Keysym = Keysym(0x1005ff01);
    /// SunFA_Tilde
    pub const SunFA_Tilde: Keysym = Keysym(0x1005ff02);
    /// SunFA_Acute
    pub const SunFA_Acute: Keysym = Keysym(0x1005ff03);
    /// SunFA_Diaeresis
    pub const SunFA_Diaeresis: Keysym = Keysym(0x1005ff04);
    /// SunFA_Cedilla
    pub const SunFA_Cedilla: Keysym = Keysym(0x1005ff05);
    /// SunF36
    pub const SunF36: Keysym = Keysym(0x1005ff10);
    /// SunF37
    pub const SunF37: Keysym = Keysym(0x1005ff11);
    /// SunSys_Req
    pub const SunSys_Req: Keysym = Keysym(0x1005ff60);
    /// SunProps
    pub const SunProps: Keysym = Keysym(0x1005ff70);
    /// SunFront
    pub const SunFront: Keysym = Keysym(0x1005ff71);
    /// SunCopy
    pub const SunCopy: Keysym = Keysym(0x1005ff72);
    /// SunOpen
    pub const SunOpen: Keysym = Keysym(0x1005ff73);
    /// SunPaste
    pub const SunPaste: Keysym = Keysym(0x1005ff74);
    /// SunCut
    pub const SunCut: Keysym = Keysym(0x1005ff75);
    /// SunPowerSwitch
    pub const SunPowerSwitch: Keysym = Keysym(0x1005ff76);
    /// SunAudioLowerVolume
    pub const SunAudioLowerVolume: Keysym = Keysym(0x1005ff77);
    /// SunAudioMute
    pub const SunAudioMute: Keysym = Keysym(0x1005ff78);
    /// SunAudioRaiseVolume
    pub const SunAudioRaiseVolume: Keysym = Keysym(0x1005ff79);
    /// SunVideoDegauss
    pub const SunVideoDegauss: Keysym = Keysym(0x1005ff7a);
    /// SunVideoLowerBrightness
    pub const SunVideoLowerBrightness: Keysym = Keysym(0x1005ff7b);
    /// SunVideoRaiseBrightness
    pub const SunVideoRaiseBrightness: Keysym = Keysym(0x1005ff7c);
    /// SunPowerSwitchShift
    pub const SunPowerSwitchShift: Keysym = Keysym(0x1005ff7d);
    /// XF86MediaPlayPause
    pub const XF86MediaPlayPause: Keysym = Keysym(0x100810a4);
    /// XF86Exit
    pub const XF86Exit: Keysym = Keysym(0x100810ae);
    /// XF86AudioBassBoost
    pub const XF86AudioBassBoost: Keysym = Keysym(0x100810d1);
    /// XF86Sport
    pub const XF86Sport: Keysym = Keysym(0x100810dc);
    /// XF86BrightnessAuto
    pub const XF86BrightnessAuto: Keysym = Keysym(0x100810f4);
    /// XF86MonBrightnessAuto
    pub const XF86MonBrightnessAuto: Keysym = Keysym(0x100810f4);
    /// XF86DisplayOff
    pub const XF86DisplayOff: Keysym = Keysym(0x100810f5);
    /// XF86OK
    pub const XF86OK: Keysym = Keysym(0x10081160);
    /// XF86GoTo
    pub const XF86GoTo: Keysym = Keysym(0x10081162);
    /// XF86Info
    pub const XF86Info: Keysym = Keysym(0x10081166);
    /// XF86VendorLogo
    pub const XF86VendorLogo: Keysym = Keysym(0x10081168);
    /// XF86MediaSelectProgramGuide
    pub const XF86MediaSelectProgramGuide: Keysym = Keysym(0x1008116a);
    /// XF86MediaSelectHome
    pub const XF86MediaSelectHome: Keysym = Keysym(0x1008116e);
    /// XF86MediaLanguageMenu
    pub const XF86MediaLanguageMenu: Keysym = Keysym(0x10081170);
    /// XF86MediaTitleMenu
    pub const XF86MediaTitleMenu: Keysym = Keysym(0x10081171);
    /// XF86AudioChannelMode
    pub const XF86AudioChannelMode: Keysym = Keysym(0x10081175);
    /// XF86AspectRatio
    pub const XF86AspectRatio: Keysym = Keysym(0x10081177);
    /// XF86MediaSelectPC
    pub const XF86MediaSelectPC: Keysym = Keysym(0x10081178);
    /// XF86MediaSelectTV
    pub const XF86MediaSelectTV: Keysym = Keysym(0x10081179);
    /// XF86MediaSelectCable
    pub const XF86MediaSelectCable: Keysym = Keysym(0x1008117a);
    /// XF86MediaSelectVCR
    pub const XF86MediaSelectVCR: Keysym = Keysym(0x1008117b);
    /// XF86MediaSelectVCRPlus
    pub const XF86MediaSelectVCRPlus: Keysym = Keysym(0x1008117c);
    /// XF86MediaSelectSatellite
    pub const XF86MediaSelectSatellite: Keysym = Keysym(0x1008117d);
    /// XF86MediaSelectTape
    pub const XF86MediaSelectTape: Keysym = Keysym(0x10081180);
    /// XF86MediaSelectRadio
    pub const XF86MediaSelectRadio: Keysym = Keysym(0x10081181);
    /// XF86MediaSelectTuner
    pub const XF86MediaSelectTuner: Keysym = Keysym(0x10081182);
    /// XF86MediaPlayer
    pub const XF86MediaPlayer: Keysym = Keysym(0x10081183);
    /// XF86MediaSelectTeletext
    pub const XF86MediaSelectTeletext: Keysym = Keysym(0x10081184);
    /// XF86DVD
    pub const XF86DVD: Keysym = Keysym(0x10081185);
    /// XF86MediaSelectDVD
    pub const XF86MediaSelectDVD: Keysym = Keysym(0x10081185);
    /// XF86MediaSelectAuxiliary
    pub const XF86MediaSelectAuxiliary: Keysym = Keysym(0x10081186);
    /// XF86Audio
    pub const XF86Audio: Keysym = Keysym(0x10081188);
    /// XF86ChannelUp
    pub const XF86ChannelUp: Keysym = Keysym(0x10081192);
    /// XF86ChannelDown
    pub const XF86ChannelDown: Keysym = Keysym(0x10081193);
    /// XF86MediaPlaySlow
    pub const XF86MediaPlaySlow: Keysym = Keysym(0x10081199);
    /// XF86Break
    pub const XF86Break: Keysym = Keysym(0x1008119b);
    /// XF86NumberEntryMode
    pub const XF86NumberEntryMode: Keysym = Keysym(0x1008119d);
    /// XF86VideoPhone
    pub const XF86VideoPhone: Keysym = Keysym(0x100811a0);
    /// XF86ZoomReset
    pub const XF86ZoomReset: Keysym = Keysym(0x100811a4);
    /// XF86Editor
    pub const XF86Editor: Keysym = Keysym(0x100811a6);
    /// XF86GraphicsEditor
    pub const XF86GraphicsEditor: Keysym = Keysym(0x100811a8);
    /// XF86Presentation
    pub const XF86Presentation: Keysym = Keysym(0x100811a9);
    /// XF86Database
    pub const XF86Database: Keysym = Keysym(0x100811aa);
    /// XF86Voicemail
    pub const XF86Voicemail: Keysym = Keysym(0x100811ac);
    /// XF86Addressbook
    pub const XF86Addressbook: Keysym = Keysym(0x100811ad);
    /// XF86DisplayToggle
    pub const XF86DisplayToggle: Keysym = Keysym(0x100811af);
    /// XF86SpellCheck
    pub const XF86SpellCheck: Keysym = Keysym(0x100811b0);
    /// XF86ContextMenu
    pub const XF86ContextMenu: Keysym = Keysym(0x100811b6);
    /// XF86MediaRepeat
    pub const XF86MediaRepeat: Keysym = Keysym(0x100811b7);
    /// XF8610ChannelsUp
    pub const XF8610ChannelsUp: Keysym = Keysym(0x100811b8);
    /// XF8610ChannelsDown
    pub const XF8610ChannelsDown: Keysym = Keysym(0x100811b9);
    /// XF86Images
    pub const XF86Images: Keysym = Keysym(0x100811ba);
    /// XF86NotificationCenter
    pub const XF86NotificationCenter: Keysym = Keysym(0x100811bc);
    /// XF86PickupPhone
    pub const XF86PickupPhone: Keysym = Keysym(0x100811bd);
    /// XF86HangupPhone
    pub const XF86HangupPhone: Keysym = Keysym(0x100811be);
    /// XF86LinkPhone
    pub const XF86LinkPhone: Keysym = Keysym(0x100811bf);
    /// XF86Fn
    pub const XF86Fn: Keysym = Keysym(0x100811d0);
    /// XF86Fn_Esc
    pub const XF86Fn_Esc: Keysym = Keysym(0x100811d1);
    /// XF86Fn_F1
    pub const XF86Fn_F1: Keysym = Keysym(0x100811d2);
    /// XF86Fn_F2
    pub const XF86Fn_F2: Keysym = Keysym(0x100811d3);
    /// XF86Fn_F3
    pub const XF86Fn_F3: Keysym = Keysym(0x100811d4);
    /// XF86Fn_F4
    pub const XF86Fn_F4: Keysym = Keysym(0x100811d5);
    /// XF86Fn_F5
    pub const XF86Fn_F5: Keysym = Keysym(0x100811d6);
    /// XF86Fn_F6
    pub const XF86Fn_F6: Keysym = Keysym(0x100811d7);
    /// XF86Fn_F7
    pub const XF86Fn_F7: Keysym = Keysym(0x100811d8);
    /// XF86Fn_F8
    pub const XF86Fn_F8: Keysym = Keysym(0x100811d9);
    /// XF86Fn_F9
    pub const XF86Fn_F9: Keysym = Keysym(0x100811da);
    /// XF86Fn_F10
    pub const XF86Fn_F10: Keysym = Keysym(0x100811db);
    /// XF86Fn_F11
    pub const XF86Fn_F11: Keysym = Keysym(0x100811dc);
    /// XF86Fn_F12
    pub const XF86Fn_F12: Keysym = Keysym(0x100811dd);
    /// XF86Fn_1
    pub const XF86Fn_1: Keysym = Keysym(0x100811de);
    /// XF86Fn_2
    pub const XF86Fn_2: Keysym = Keysym(0x100811df);
    /// XF86Fn_D
    pub const XF86Fn_D: Keysym = Keysym(0x100811e0);
    /// XF86Fn_E
    pub const XF86Fn_E: Keysym = Keysym(0x100811e1);
    /// XF86Fn_F
    pub const XF86Fn_F: Keysym = Keysym(0x100811e2);
    /// XF86Fn_S
    pub const XF86Fn_S: Keysym = Keysym(0x100811e3);
    /// XF86Fn_B
    pub const XF86Fn_B: Keysym = Keysym(0x100811e4);
    /// XF86FnRightShift
    pub const XF86FnRightShift: Keysym = Keysym(0x100811e5);
    /// XF86Numeric0
    pub const XF86Numeric0: Keysym = Keysym(0x10081200);
    /// XF86Numeric1
    pub const XF86Numeric1: Keysym = Keysym(0x10081201);
    /// XF86Numeric2
    pub const XF86Numeric2: Keysym = Keysym(0x10081202);
    /// XF86Numeric3
    pub const XF86Numeric3: Keysym = Keysym(0x10081203);
    /// XF86Numeric4
    pub const XF86Numeric4: Keysym = Keysym(0x10081204);
    /// XF86Numeric5
    pub const XF86Numeric5: Keysym = Keysym(0x10081205);
    /// XF86Numeric6
    pub const XF86Numeric6: Keysym = Keysym(0x10081206);
    /// XF86Numeric7
    pub const XF86Numeric7: Keysym = Keysym(0x10081207);
    /// XF86Numeric8
    pub const XF86Numeric8: Keysym = Keysym(0x10081208);
    /// XF86Numeric9
    pub const XF86Numeric9: Keysym = Keysym(0x10081209);
    /// XF86NumericStar
    pub const XF86NumericStar: Keysym = Keysym(0x1008120a);
    /// XF86NumericPound
    pub const XF86NumericPound: Keysym = Keysym(0x1008120b);
    /// XF86NumericA
    pub const XF86NumericA: Keysym = Keysym(0x1008120c);
    /// XF86NumericB
    pub const XF86NumericB: Keysym = Keysym(0x1008120d);
    /// XF86NumericC
    pub const XF86NumericC: Keysym = Keysym(0x1008120e);
    /// XF86NumericD
    pub const XF86NumericD: Keysym = Keysym(0x1008120f);
    /// XF86CameraFocus
    pub const XF86CameraFocus: Keysym = Keysym(0x10081210);
    /// XF86WPSButton
    pub const XF86WPSButton: Keysym = Keysym(0x10081211);
    /// XF86CameraZoomIn
    pub const XF86CameraZoomIn: Keysym = Keysym(0x10081215);
    /// XF86CameraZoomOut
    pub const XF86CameraZoomOut: Keysym = Keysym(0x10081216);
    /// XF86CameraUp
    pub const XF86CameraUp: Keysym = Keysym(0x10081217);
    /// XF86CameraDown
    pub const XF86CameraDown: Keysym = Keysym(0x10081218);
    /// XF86CameraLeft
    pub const XF86CameraLeft: Keysym = Keysym(0x10081219);
    /// XF86CameraRight
    pub const XF86CameraRight: Keysym = Keysym(0x1008121a);
    /// XF86AttendantOn
    pub const XF86AttendantOn: Keysym = Keysym(0x1008121b);
    /// XF86AttendantOff
    pub const XF86AttendantOff: Keysym = Keysym(0x1008121c);
    /// XF86AttendantToggle
    pub const XF86AttendantToggle: Keysym = Keysym(0x1008121d);
    /// XF86LightsToggle
    pub const XF86LightsToggle: Keysym = Keysym(0x1008121e);
    /// XF86ALSToggle
    pub const XF86ALSToggle: Keysym = Keysym(0x10081230);
    /// XF86RefreshRateToggle
    pub const XF86RefreshRateToggle: Keysym = Keysym(0x10081232);
    /// XF86Buttonconfig
    pub const XF86Buttonconfig: Keysym = Keysym(0x10081240);
    /// XF86Taskmanager
    pub const XF86Taskmanager: Keysym = Keysym(0x10081241);
    /// XF86Journal
    pub const XF86Journal: Keysym = Keysym(0x10081242);
    /// XF86ControlPanel
    pub const XF86ControlPanel: Keysym = Keysym(0x10081243);
    /// XF86AppSelect
    pub const XF86AppSelect: Keysym = Keysym(0x10081244);
    /// XF86Screensaver
    pub const XF86Screensaver: Keysym = Keysym(0x10081245);
    /// XF86VoiceCommand
    pub const XF86VoiceCommand: Keysym = Keysym(0x10081246);
    /// XF86Assistant
    pub const XF86Assistant: Keysym = Keysym(0x10081247);
    /// XF86EmojiPicker
    pub const XF86EmojiPicker: Keysym = Keysym(0x10081249);
    /// XF86Dictate
    pub const XF86Dictate: Keysym = Keysym(0x1008124a);
    /// XF86CameraAccessEnable
    pub const XF86CameraAccessEnable: Keysym = Keysym(0x1008124b);
    /// XF86CameraAccessDisable
    pub const XF86CameraAccessDisable: Keysym = Keysym(0x1008124c);
    /// XF86CameraAccessToggle
    pub const XF86CameraAccessToggle: Keysym = Keysym(0x1008124d);
    /// XF86Accessibility
    pub const XF86Accessibility: Keysym = Keysym(0x1008124e);
    /// XF86DoNotDisturb
    pub const XF86DoNotDisturb: Keysym = Keysym(0x1008124f);
    /// XF86BrightnessMin
    pub const XF86BrightnessMin: Keysym = Keysym(0x10081250);
    /// XF86BrightnessMax
    pub const XF86BrightnessMax: Keysym = Keysym(0x10081251);
    /// XF86KbdInputAssistPrev
    pub const XF86KbdInputAssistPrev: Keysym = Keysym(0x10081260);
    /// XF86KbdInputAssistNext
    pub const XF86KbdInputAssistNext: Keysym = Keysym(0x10081261);
    /// XF86KbdInputAssistPrevgroup
    pub const XF86KbdInputAssistPrevgroup: Keysym = Keysym(0x10081262);
    /// XF86KbdInputAssistNextgroup
    pub const XF86KbdInputAssistNextgroup: Keysym = Keysym(0x10081263);
    /// XF86KbdInputAssistAccept
    pub const XF86KbdInputAssistAccept: Keysym = Keysym(0x10081264);
    /// XF86KbdInputAssistCancel
    pub const XF86KbdInputAssistCancel: Keysym = Keysym(0x10081265);
    /// XF86RightUp
    pub const XF86RightUp: Keysym = Keysym(0x10081266);
    /// XF86RightDown
    pub const XF86RightDown: Keysym = Keysym(0x10081267);
    /// XF86LeftUp
    pub const XF86LeftUp: Keysym = Keysym(0x10081268);
    /// XF86LeftDown
    pub const XF86LeftDown: Keysym = Keysym(0x10081269);
    /// XF86RootMenu
    pub const XF86RootMenu: Keysym = Keysym(0x1008126a);
    /// XF86MediaTopMenu
    pub const XF86MediaTopMenu: Keysym = Keysym(0x1008126b);
    /// XF86Numeric11
    pub const XF86Numeric11: Keysym = Keysym(0x1008126c);
    /// XF86Numeric12
    pub const XF86Numeric12: Keysym = Keysym(0x1008126d);
    /// XF86AudioDesc
    pub const XF86AudioDesc: Keysym = Keysym(0x1008126e);
    /// XF863DMode
    pub const XF863DMode: Keysym = Keysym(0x1008126f);
    /// XF86NextFavorite
    pub const XF86NextFavorite: Keysym = Keysym(0x10081270);
    /// XF86StopRecord
    pub const XF86StopRecord: Keysym = Keysym(0x10081271);
    /// XF86PauseRecord
    pub const XF86PauseRecord: Keysym = Keysym(0x10081272);
    /// XF86VOD
    pub const XF86VOD: Keysym = Keysym(0x10081273);
    /// XF86Unmute
    pub const XF86Unmute: Keysym = Keysym(0x10081274);
    /// XF86FastReverse
    pub const XF86FastReverse: Keysym = Keysym(0x10081275);
    /// XF86SlowReverse
    pub const XF86SlowReverse: Keysym = Keysym(0x10081276);
    /// XF86Data
    pub const XF86Data: Keysym = Keysym(0x10081277);
    /// XF86OnScreenKeyboard
    pub const XF86OnScreenKeyboard: Keysym = Keysym(0x10081278);
    /// XF86PrivacyScreenToggle
    pub const XF86PrivacyScreenToggle: Keysym = Keysym(0x10081279);
    /// XF86SelectiveScreenshot
    pub const XF86SelectiveScreenshot: Keysym = Keysym(0x1008127a);
    /// XF86NextElement
    pub const XF86NextElement: Keysym = Keysym(0x1008127b);
    /// XF86PreviousElement
    pub const XF86PreviousElement: Keysym = Keysym(0x1008127c);
    /// XF86AutopilotEngageToggle
    pub const XF86AutopilotEngageToggle: Keysym = Keysym(0x1008127d);
    /// XF86MarkWaypoint
    pub const XF86MarkWaypoint: Keysym = Keysym(0x1008127e);
    /// XF86Sos
    pub const XF86Sos: Keysym = Keysym(0x1008127f);
    /// XF86NavChart
    pub const XF86NavChart: Keysym = Keysym(0x10081280);
    /// XF86FishingChart
    pub const XF86FishingChart: Keysym = Keysym(0x10081281);
    /// XF86SingleRangeRadar
    pub const XF86SingleRangeRadar: Keysym = Keysym(0x10081282);
    /// XF86DualRangeRadar
    pub const XF86DualRangeRadar: Keysym = Keysym(0x10081283);
    /// XF86RadarOverlay
    pub const XF86RadarOverlay: Keysym = Keysym(0x10081284);
    /// XF86TraditionalSonar
    pub const XF86TraditionalSonar: Keysym = Keysym(0x10081285);
    /// XF86ClearvuSonar
    pub const XF86ClearvuSonar: Keysym = Keysym(0x10081286);
    /// XF86SidevuSonar
    pub const XF86SidevuSonar: Keysym = Keysym(0x10081287);
    /// XF86NavInfo
    pub const XF86NavInfo: Keysym = Keysym(0x10081288);
    /// XF86Macro1
    pub const XF86Macro1: Keysym = Keysym(0x10081290);
    /// XF86Macro2
    pub const XF86Macro2: Keysym = Keysym(0x10081291);
    /// XF86Macro3
    pub const XF86Macro3: Keysym = Keysym(0x10081292);
    /// XF86Macro4
    pub const XF86Macro4: Keysym = Keysym(0x10081293);
    /// XF86Macro5
    pub const XF86Macro5: Keysym = Keysym(0x10081294);
    /// XF86Macro6
    pub const XF86Macro6: Keysym = Keysym(0x10081295);
    /// XF86Macro7
    pub const XF86Macro7: Keysym = Keysym(0x10081296);
    /// XF86Macro8
    pub const XF86Macro8: Keysym = Keysym(0x10081297);
    /// XF86Macro9
    pub const XF86Macro9: Keysym = Keysym(0x10081298);
    /// XF86Macro10
    pub const XF86Macro10: Keysym = Keysym(0x10081299);
    /// XF86Macro11
    pub const XF86Macro11: Keysym = Keysym(0x1008129a);
    /// XF86Macro12
    pub const XF86Macro12: Keysym = Keysym(0x1008129b);
    /// XF86Macro13
    pub const XF86Macro13: Keysym = Keysym(0x1008129c);
    /// XF86Macro14
    pub const XF86Macro14: Keysym = Keysym(0x1008129d);
    /// XF86Macro15
    pub const XF86Macro15: Keysym = Keysym(0x1008129e);
    /// XF86Macro16
    pub const XF86Macro16: Keysym = Keysym(0x1008129f);
    /// XF86Macro17
    pub const XF86Macro17: Keysym = Keysym(0x100812a0);
    /// XF86Macro18
    pub const XF86Macro18: Keysym = Keysym(0x100812a1);
    /// XF86Macro19
    pub const XF86Macro19: Keysym = Keysym(0x100812a2);
    /// XF86Macro20
    pub const XF86Macro20: Keysym = Keysym(0x100812a3);
    /// XF86Macro21
    pub const XF86Macro21: Keysym = Keysym(0x100812a4);
    /// XF86Macro22
    pub const XF86Macro22: Keysym = Keysym(0x100812a5);
    /// XF86Macro23
    pub const XF86Macro23: Keysym = Keysym(0x100812a6);
    /// XF86Macro24
    pub const XF86Macro24: Keysym = Keysym(0x100812a7);
    /// XF86Macro25
    pub const XF86Macro25: Keysym = Keysym(0x100812a8);
    /// XF86Macro26
    pub const XF86Macro26: Keysym = Keysym(0x100812a9);
    /// XF86Macro27
    pub const XF86Macro27: Keysym = Keysym(0x100812aa);
    /// XF86Macro28
    pub const XF86Macro28: Keysym = Keysym(0x100812ab);
    /// XF86Macro29
    pub const XF86Macro29: Keysym = Keysym(0x100812ac);
    /// XF86Macro30
    pub const XF86Macro30: Keysym = Keysym(0x100812ad);
    /// XF86MacroRecordStart
    pub const XF86MacroRecordStart: Keysym = Keysym(0x100812b0);
    /// XF86MacroRecordStop
    pub const XF86MacroRecordStop: Keysym = Keysym(0x100812b1);
    /// XF86MacroPresetCycle
    pub const XF86MacroPresetCycle: Keysym = Keysym(0x100812b2);
    /// XF86MacroPreset1
    pub const XF86MacroPreset1: Keysym = Keysym(0x100812b3);
    /// XF86MacroPreset2
    pub const XF86MacroPreset2: Keysym = Keysym(0x100812b4);
    /// XF86MacroPreset3
    pub const XF86MacroPreset3: Keysym = Keysym(0x100812b5);
    /// XF86KbdLcdMenu1
    pub const XF86KbdLcdMenu1: Keysym = Keysym(0x100812b8);
    /// XF86KbdLcdMenu2
    pub const XF86KbdLcdMenu2: Keysym = Keysym(0x100812b9);
    /// XF86KbdLcdMenu3
    pub const XF86KbdLcdMenu3: Keysym = Keysym(0x100812ba);
    /// XF86KbdLcdMenu4
    pub const XF86KbdLcdMenu4: Keysym = Keysym(0x100812bb);
    /// XF86KbdLcdMenu5
    pub const XF86KbdLcdMenu5: Keysym = Keysym(0x100812bc);
    /// XF86PerformanceMode
    pub const XF86PerformanceMode: Keysym = Keysym(0x100812bd);
    /// XF86Switch_VT_1
    pub const XF86Switch_VT_1: Keysym = Keysym(0x1008fe01);
    /// XF86Switch_VT_2
    pub const XF86Switch_VT_2: Keysym = Keysym(0x1008fe02);
    /// XF86Switch_VT_3
    pub const XF86Switch_VT_3: Keysym = Keysym(0x1008fe03);
    /// XF86Switch_VT_4
    pub const XF86Switch_VT_4: Keysym = Keysym(0x1008fe04);
    /// XF86Switch_VT_5
    pub const XF86Switch_VT_5: Keysym = Keysym(0x1008fe05);
    /// XF86Switch_VT_6
    pub const XF86Switch_VT_6: Keysym = Keysym(0x1008fe06);
    /// XF86Switch_VT_7
    pub const XF86Switch_VT_7: Keysym = Keysym(0x1008fe07);
    /// XF86Switch_VT_8
    pub const XF86Switch_VT_8: Keysym = Keysym(0x1008fe08);
    /// XF86Switch_VT_9
    pub const XF86Switch_VT_9: Keysym = Keysym(0x1008fe09);
    /// XF86Switch_VT_10
    pub const XF86Switch_VT_10: Keysym = Keysym(0x1008fe0a);
    /// XF86Switch_VT_11
    pub const XF86Switch_VT_11: Keysym = Keysym(0x1008fe0b);
    /// XF86Switch_VT_12
    pub const XF86Switch_VT_12: Keysym = Keysym(0x1008fe0c);
    /// XF86Ungrab
    pub const XF86Ungrab: Keysym = Keysym(0x1008fe20);
    /// XF86ClearGrab
    pub const XF86ClearGrab: Keysym = Keysym(0x1008fe21);
    /// XF86Next_VMode
    pub const XF86Next_VMode: Keysym = Keysym(0x1008fe22);
    /// XF86Prev_VMode
    pub const XF86Prev_VMode: Keysym = Keysym(0x1008fe23);
    /// XF86LogWindowTree
    pub const XF86LogWindowTree: Keysym = Keysym(0x1008fe24);
    /// XF86LogGrabInfo
    pub const XF86LogGrabInfo: Keysym = Keysym(0x1008fe25);
    /// XF86ModeLock
    pub const XF86ModeLock: Keysym = Keysym(0x1008ff01);
    /// XF86MonBrightnessUp
    pub const XF86MonBrightnessUp: Keysym = Keysym(0x1008ff02);
    /// XF86MonBrightnessDown
    pub const XF86MonBrightnessDown: Keysym = Keysym(0x1008ff03);
    /// XF86KbdLightOnOff
    pub const XF86KbdLightOnOff: Keysym = Keysym(0x1008ff04);
    /// XF86KbdBrightnessUp
    pub const XF86KbdBrightnessUp: Keysym = Keysym(0x1008ff05);
    /// XF86KbdBrightnessDown
    pub const XF86KbdBrightnessDown: Keysym = Keysym(0x1008ff06);
    /// XF86MonBrightnessCycle
    pub const XF86MonBrightnessCycle: Keysym = Keysym(0x1008ff07);
    /// XF86Standby
    pub const XF86Standby: Keysym = Keysym(0x1008ff10);
    /// XF86AudioLowerVolume
    pub const XF86AudioLowerVolume: Keysym = Keysym(0x1008ff11);
    /// XF86AudioMute
    pub const XF86AudioMute: Keysym = Keysym(0x1008ff12);
    /// XF86AudioRaiseVolume
    pub const XF86AudioRaiseVolume: Keysym = Keysym(0x1008ff13);
    /// XF86AudioPlay
    pub const XF86AudioPlay: Keysym = Keysym(0x1008ff14);
    /// XF86AudioStop
    pub const XF86AudioStop: Keysym = Keysym(0x1008ff15);
    /// XF86AudioPrev
    pub const XF86AudioPrev: Keysym = Keysym(0x1008ff16);
    /// XF86AudioNext
    pub const XF86AudioNext: Keysym = Keysym(0x1008ff17);
    /// XF86HomePage
    pub const XF86HomePage: Keysym = Keysym(0x1008ff18);
    /// XF86Mail
    pub const XF86Mail: Keysym = Keysym(0x1008ff19);
    /// XF86Start
    pub const XF86Start: Keysym = Keysym(0x1008ff1a);
    /// XF86Search
    pub const XF86Search: Keysym = Keysym(0x1008ff1b);
    /// XF86AudioRecord
    pub const XF86AudioRecord: Keysym = Keysym(0x1008ff1c);
    /// XF86Calculator
    pub const XF86Calculator: Keysym = Keysym(0x1008ff1d);
    /// XF86Memo
    pub const XF86Memo: Keysym = Keysym(0x1008ff1e);
    /// XF86ToDoList
    pub const XF86ToDoList: Keysym = Keysym(0x1008ff1f);
    /// XF86Calendar
    pub const XF86Calendar: Keysym = Keysym(0x1008ff20);
    /// XF86PowerDown
    pub const XF86PowerDown: Keysym = Keysym(0x1008ff21);
    /// XF86ContrastAdjust
    pub const XF86ContrastAdjust: Keysym = Keysym(0x1008ff22);
    /// XF86RockerUp
    pub const XF86RockerUp: Keysym = Keysym(0x1008ff23);
    /// XF86RockerDown
    pub const XF86RockerDown: Keysym = Keysym(0x1008ff24);
    /// XF86RockerEnter
    pub const XF86RockerEnter: Keysym = Keysym(0x1008ff25);
    /// XF86Back
    pub const XF86Back: Keysym = Keysym(0x1008ff26);
    /// XF86Forward
    pub const XF86Forward: Keysym = Keysym(0x1008ff27);
    /// XF86Stop
    pub const XF86Stop: Keysym = Keysym(0x1008ff28);
    /// XF86Refresh
    pub const XF86Refresh: Keysym = Keysym(0x1008ff29);
    /// XF86PowerOff
    pub const XF86PowerOff: Keysym = Keysym(0x1008ff2a);
    /// XF86WakeUp
    pub const XF86WakeUp: Keysym = Keysym(0x1008ff2b);
    /// XF86Eject
    pub const XF86Eject: Keysym = Keysym(0x1008ff2c);
    /// XF86ScreenSaver
    pub const XF86ScreenSaver: Keysym = Keysym(0x1008ff2d);
    /// XF86WWW
    pub const XF86WWW: Keysym = Keysym(0x1008ff2e);
    /// XF86Sleep
    pub const XF86Sleep: Keysym = Keysym(0x1008ff2f);
    /// XF86Favorites
    pub const XF86Favorites: Keysym = Keysym(0x1008ff30);
    /// XF86AudioPause
    pub const XF86AudioPause: Keysym = Keysym(0x1008ff31);
    /// XF86AudioMedia
    pub const XF86AudioMedia: Keysym = Keysym(0x1008ff32);
    /// XF86MyComputer
    pub const XF86MyComputer: Keysym = Keysym(0x1008ff33);
    /// XF86VendorHome
    pub const XF86VendorHome: Keysym = Keysym(0x1008ff34);
    /// XF86LightBulb
    pub const XF86LightBulb: Keysym = Keysym(0x1008ff35);
    /// XF86Shop
    pub const XF86Shop: Keysym = Keysym(0x1008ff36);
    /// XF86History
    pub const XF86History: Keysym = Keysym(0x1008ff37);
    /// XF86OpenURL
    pub const XF86OpenURL: Keysym = Keysym(0x1008ff38);
    /// XF86AddFavorite
    pub const XF86AddFavorite: Keysym = Keysym(0x1008ff39);
    /// XF86HotLinks
    pub const XF86HotLinks: Keysym = Keysym(0x1008ff3a);
    /// XF86BrightnessAdjust
    pub const XF86BrightnessAdjust: Keysym = Keysym(0x1008ff3b);
    /// XF86Finance
    pub const XF86Finance: Keysym = Keysym(0x1008ff3c);
    /// XF86Community
    pub const XF86Community: Keysym = Keysym(0x1008ff3d);
    /// XF86AudioRewind
    pub const XF86AudioRewind: Keysym = Keysym(0x1008ff3e);
    /// XF86BackForward
    pub const XF86BackForward: Keysym = Keysym(0x1008ff3f);
    /// XF86Launch0
    pub const XF86Launch0: Keysym = Keysym(0x1008ff40);
    /// XF86Launch1
    pub const XF86Launch1: Keysym = Keysym(0x1008ff41);
    /// XF86Launch2
    pub const XF86Launch2: Keysym = Keysym(0x1008ff42);
    /// XF86Launch3
    pub const XF86Launch3: Keysym = Keysym(0x1008ff43);
    /// XF86Launch4
    pub const XF86Launch4: Keysym = Keysym(0x1008ff44);
    /// XF86Launch5
    pub const XF86Launch5: Keysym = Keysym(0x1008ff45);
    /// XF86Launch6
    pub const XF86Launch6: Keysym = Keysym(0x1008ff46);
    /// XF86Launch7
    pub const XF86Launch7: Keysym = Keysym(0x1008ff47);
    /// XF86Launch8
    pub const XF86Launch8: Keysym = Keysym(0x1008ff48);
    /// XF86Launch9
    pub const XF86Launch9: Keysym = Keysym(0x1008ff49);
    /// XF86LaunchA
    pub const XF86LaunchA: Keysym = Keysym(0x1008ff4a);
    /// XF86LaunchB
    pub const XF86LaunchB: Keysym = Keysym(0x1008ff4b);
    /// XF86LaunchC
    pub const XF86LaunchC: Keysym = Keysym(0x1008ff4c);
    /// XF86LaunchD
    pub const XF86LaunchD: Keysym = Keysym(0x1008ff4d);
    /// XF86LaunchE
    pub const XF86LaunchE: Keysym = Keysym(0x1008ff4e);
    /// XF86LaunchF
    pub const XF86LaunchF: Keysym = Keysym(0x1008ff4f);
    /// XF86ApplicationLeft
    pub const XF86ApplicationLeft: Keysym = Keysym(0x1008ff50);
    /// XF86ApplicationRight
    pub const XF86ApplicationRight: Keysym = Keysym(0x1008ff51);
    /// XF86Book
    pub const XF86Book: Keysym = Keysym(0x1008ff52);
    /// XF86CD
    pub const XF86CD: Keysym = Keysym(0x1008ff53);
    /// XF86MediaSelectCD
    pub const XF86MediaSelectCD: Keysym = Keysym(0x1008ff53);
    /// XF86Calculater
    pub const XF86Calculater: Keysym = Keysym(0x1008ff54);
    /// XF86Clear
    pub const XF86Clear: Keysym = Keysym(0x1008ff55);
    /// XF86Close
    pub const XF86Close: Keysym = Keysym(0x1008ff56);
    /// XF86Copy
    pub const XF86Copy: Keysym = Keysym(0x1008ff57);
    /// XF86Cut
    pub const XF86Cut: Keysym = Keysym(0x1008ff58);
    /// XF86Display
    pub const XF86Display: Keysym = Keysym(0x1008ff59);
    /// XF86DOS
    pub const XF86DOS: Keysym = Keysym(0x1008ff5a);
    /// XF86Documents
    pub const XF86Documents: Keysym = Keysym(0x1008ff5b);
    /// XF86Excel
    pub const XF86Excel: Keysym = Keysym(0x1008ff5c);
    /// XF86Explorer
    pub const XF86Explorer: Keysym = Keysym(0x1008ff5d);
    /// XF86Game
    pub const XF86Game: Keysym = Keysym(0x1008ff5e);
    /// XF86Go
    pub const XF86Go: Keysym = Keysym(0x1008ff5f);
    /// XF86iTouch
    pub const XF86iTouch: Keysym = Keysym(0x1008ff60);
    /// XF86LogOff
    pub const XF86LogOff: Keysym = Keysym(0x1008ff61);
    /// XF86Market
    pub const XF86Market: Keysym = Keysym(0x1008ff62);
    /// XF86Meeting
    pub const XF86Meeting: Keysym = Keysym(0x1008ff63);
    /// XF86MenuKB
    pub const XF86MenuKB: Keysym = Keysym(0x1008ff65);
    /// XF86MenuPB
    pub const XF86MenuPB: Keysym = Keysym(0x1008ff66);
    /// XF86MySites
    pub const XF86MySites: Keysym = Keysym(0x1008ff67);
    /// XF86New
    pub const XF86New: Keysym = Keysym(0x1008ff68);
    /// XF86News
    pub const XF86News: Keysym = Keysym(0x1008ff69);
    /// XF86OfficeHome
    pub const XF86OfficeHome: Keysym = Keysym(0x1008ff6a);
    /// XF86Open
    pub const XF86Open: Keysym = Keysym(0x1008ff6b);
    /// XF86Option
    pub const XF86Option: Keysym = Keysym(0x1008ff6c);
    /// XF86Paste
    pub const XF86Paste: Keysym = Keysym(0x1008ff6d);
    /// XF86Phone
    pub const XF86Phone: Keysym = Keysym(0x1008ff6e);
    /// XF86Q
    pub const XF86Q: Keysym = Keysym(0x1008ff70);
    /// XF86Reply
    pub const XF86Reply: Keysym = Keysym(0x1008ff72);
    /// XF86Reload
    pub const XF86Reload: Keysym = Keysym(0x1008ff73);
    /// XF86RotateWindows
    pub const XF86RotateWindows: Keysym = Keysym(0x1008ff74);
    /// XF86RotationPB
    pub const XF86RotationPB: Keysym = Keysym(0x1008ff75);
    /// XF86RotationKB
    pub const XF86RotationKB: Keysym = Keysym(0x1008ff76);
    /// XF86Save
    pub const XF86Save: Keysym = Keysym(0x1008ff77);
    /// XF86ScrollUp
    pub const XF86ScrollUp: Keysym = Keysym(0x1008ff78);
    /// XF86ScrollDown
    pub const XF86ScrollDown: Keysym = Keysym(0x1008ff79);
    /// XF86ScrollClick
    pub const XF86ScrollClick: Keysym = Keysym(0x1008ff7a);
    /// XF86Send
    pub const XF86Send: Keysym = Keysym(0x1008ff7b);
    /// XF86Spell
    pub const XF86Spell: Keysym = Keysym(0x1008ff7c);
    /// XF86SplitScreen
    pub const XF86SplitScreen: Keysym = Keysym(0x1008ff7d);
    /// XF86Support
    pub const XF86Support: Keysym = Keysym(0x1008ff7e);
    /// XF86TaskPane
    pub const XF86TaskPane: Keysym = Keysym(0x1008ff7f);
    /// XF86Terminal
    pub const XF86Terminal: Keysym = Keysym(0x1008ff80);
    /// XF86Tools
    pub const XF86Tools: Keysym = Keysym(0x1008ff81);
    /// XF86Travel
    pub const XF86Travel: Keysym = Keysym(0x1008ff82);
    /// XF86UserPB
    pub const XF86UserPB: Keysym = Keysym(0x1008ff84);
    /// XF86User1KB
    pub const XF86User1KB: Keysym = Keysym(0x1008ff85);
    /// XF86User2KB
    pub const XF86User2KB: Keysym = Keysym(0x1008ff86);
    /// XF86Video
    pub const XF86Video: Keysym = Keysym(0x1008ff87);
    /// XF86WheelButton
    pub const XF86WheelButton: Keysym = Keysym(0x1008ff88);
    /// XF86Word
    pub const XF86Word: Keysym = Keysym(0x1008ff89);
    /// XF86Xfer
    pub const XF86Xfer: Keysym = Keysym(0x1008ff8a);
    /// XF86ZoomIn
    pub const XF86ZoomIn: Keysym = Keysym(0x1008ff8b);
    /// XF86ZoomOut
    pub const XF86ZoomOut: Keysym = Keysym(0x1008ff8c);
    /// XF86Away
    pub const XF86Away: Keysym = Keysym(0x1008ff8d);
    /// XF86Messenger
    pub const XF86Messenger: Keysym = Keysym(0x1008ff8e);
    /// XF86WebCam
    pub const XF86WebCam: Keysym = Keysym(0x1008ff8f);
    /// XF86MailForward
    pub const XF86MailForward: Keysym = Keysym(0x1008ff90);
    /// XF86Pictures
    pub const XF86Pictures: Keysym = Keysym(0x1008ff91);
    /// XF86Music
    pub const XF86Music: Keysym = Keysym(0x1008ff92);
    /// XF86Battery
    pub const XF86Battery: Keysym = Keysym(0x1008ff93);
    /// XF86Bluetooth
    pub const XF86Bluetooth: Keysym = Keysym(0x1008ff94);
    /// XF86WLAN
    pub const XF86WLAN: Keysym = Keysym(0x1008ff95);
    /// XF86UWB
    pub const XF86UWB: Keysym = Keysym(0x1008ff96);
    /// XF86AudioForward
    pub const XF86AudioForward: Keysym = Keysym(0x1008ff97);
    /// XF86AudioRepeat
    pub const XF86AudioRepeat: Keysym = Keysym(0x1008ff98);
    /// XF86AudioRandomPlay
    pub const XF86AudioRandomPlay: Keysym = Keysym(0x1008ff99);
    /// XF86Subtitle
    pub const XF86Subtitle: Keysym = Keysym(0x1008ff9a);
    /// XF86AudioCycleTrack
    pub const XF86AudioCycleTrack: Keysym = Keysym(0x1008ff9b);
    /// XF86CycleAngle
    pub const XF86CycleAngle: Keysym = Keysym(0x1008ff9c);
    /// XF86FrameBack
    pub const XF86FrameBack: Keysym = Keysym(0x1008ff9d);
    /// XF86FrameForward
    pub const XF86FrameForward: Keysym = Keysym(0x1008ff9e);
    /// XF86Time
    pub const XF86Time: Keysym = Keysym(0x1008ff9f);
    /// XF86Select
    pub const XF86Select: Keysym = Keysym(0x1008ffa0);
    /// XF86View
    pub const XF86View: Keysym = Keysym(0x1008ffa1);
    /// XF86TopMenu
    pub const XF86TopMenu: Keysym = Keysym(0x1008ffa2);
    /// XF86Red
    pub const XF86Red: Keysym = Keysym(0x1008ffa3);
    /// XF86Green
    pub const XF86Green: Keysym = Keysym(0x1008ffa4);
    /// XF86Yellow
    pub const XF86Yellow: Keysym = Keysym(0x1008ffa5);
    /// XF86Blue
    pub const XF86Blue: Keysym = Keysym(0x1008ffa6);
    /// XF86Suspend
    pub const XF86Suspend: Keysym = Keysym(0x1008ffa7);
    /// XF86Hibernate
    pub const XF86Hibernate: Keysym = Keysym(0x1008ffa8);
    /// XF86TouchpadToggle
    pub const XF86TouchpadToggle: Keysym = Keysym(0x1008ffa9);
    /// XF86TouchpadOn
    pub const XF86TouchpadOn: Keysym = Keysym(0x1008ffb0);
    /// XF86TouchpadOff
    pub const XF86TouchpadOff: Keysym = Keysym(0x1008ffb1);
    /// XF86AudioMicMute
    pub const XF86AudioMicMute: Keysym = Keysym(0x1008ffb2);
    /// XF86Keyboard
    pub const XF86Keyboard: Keysym = Keysym(0x1008ffb3);
    /// XF86WWAN
    pub const XF86WWAN: Keysym = Keysym(0x1008ffb4);
    /// XF86RFKill
    pub const XF86RFKill: Keysym = Keysym(0x1008ffb5);
    /// XF86AudioPreset
    pub const XF86AudioPreset: Keysym = Keysym(0x1008ffb6);
    /// XF86RotationLockToggle
    pub const XF86RotationLockToggle: Keysym = Keysym(0x1008ffb7);
    /// XF86FullScreen
    pub const XF86FullScreen: Keysym = Keysym(0x1008ffb8);
}

