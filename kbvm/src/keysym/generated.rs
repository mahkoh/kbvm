use super::*;

#[cfg(test)]
pub(super) const LEN: usize = 2496;

pub(super) const LONGEST_NAME: usize = 27;

pub(super) static NAMES: &str = "NoSymbolspaceexclamquotedblnumbersigndollarpercentampersandapostrophequoterightparenleftparenrightasteriskpluscommaminusperiodslash0123456789colonsemicolonlessequalgreaterquestionatABCDEFGHIJKLMNOPQRSTUVWXYZbracketleftbackslashbracketrightasciicircumunderscoregravequoteleftabcdefghijklmnopqrstuvwxyzbraceleftbarbracerightasciitildenobreakspaceexclamdowncentsterlingcurrencyyenbrokenbarsectiondiaeresiscopyrightordfeminineguillemotleftguillemetleftnotsignhyphenregisteredmacrondegreeplusminustwosuperiorthreesuperioracutemuparagraphperiodcenteredcedillaonesuperiormasculineordmasculineguillemotrightguillemetrightonequarteronehalfthreequartersquestiondownAgraveAacuteAcircumflexAtildeAdiaeresisAringAECcedillaEgraveEacuteEcircumflexEdiaeresisIgraveIacuteIcircumflexIdiaeresisETHEthNtildeOgraveOacuteOcircumflexOtildeOdiaeresismultiplyOslashOobliqueUgraveUacuteUcircumflexUdiaeresisYacuteTHORNThornssharpagraveaacuteacircumflexatildeadiaeresisaringaeccedillaegraveeacuteecircumflexediaeresisigraveiacuteicircumflexidiaeresisethntildeograveoacuteocircumflexotildeodiaeresisdivisionoslashoobliqueugraveuacuteucircumflexudiaeresisyacutethornydiaeresisAogonekbreveLstrokeLcaronSacuteScaronScedillaTcaronZacuteZcaronZabovedotaogonekogoneklstrokelcaronsacutecaronscaronscedillatcaronzacutedoubleacutezcaronzabovedotRacuteAbreveLacuteCacuteCcaronEogonekEcaronDcaronDstrokeNacuteNcaronOdoubleacuteRcaronUringUdoubleacuteTcedillaracuteabrevelacutecacuteccaroneogonekecarondcarondstrokenacutencaronodoubleacutercaronuringudoubleacutetcedillaabovedotHstrokeHcircumflexIabovedotGbreveJcircumflexhstrokehcircumflexidotlessgbrevejcircumflexCabovedotCcircumflexGabovedotGcircumflexUbreveScircumflexcabovedotccircumflexgabovedotgcircumflexubrevescircumflexkrakappaRcedillaItildeLcedillaEmacronGcedillaTslashrcedillaitildelcedillaemacrongcedillatslashENGengAmacronIogonekEabovedotImacronNcedillaOmacronKcedillaUogonekUtildeUmacronamacroniogonekeabovedotimacronncedillaomacronkcedillauogonekutildeumacronoverlinekana_fullstopkana_openingbracketkana_closingbracketkana_commakana_conjunctivekana_middledotkana_WOkana_akana_ikana_ukana_ekana_okana_yakana_yukana_yokana_tsukana_tuprolongedsoundkana_Akana_Ikana_Ukana_Ekana_Okana_KAkana_KIkana_KUkana_KEkana_KOkana_SAkana_SHIkana_SUkana_SEkana_SOkana_TAkana_CHIkana_TIkana_TSUkana_TUkana_TEkana_TOkana_NAkana_NIkana_NUkana_NEkana_NOkana_HAkana_HIkana_FUkana_HUkana_HEkana_HOkana_MAkana_MIkana_MUkana_MEkana_MOkana_YAkana_YUkana_YOkana_RAkana_RIkana_RUkana_REkana_ROkana_WAkana_NvoicedsoundsemivoicedsoundArabic_commaArabic_semicolonArabic_question_markArabic_hamzaArabic_maddaonalefArabic_hamzaonalefArabic_hamzaonwawArabic_hamzaunderalefArabic_hamzaonyehArabic_alefArabic_behArabic_tehmarbutaArabic_tehArabic_thehArabic_jeemArabic_hahArabic_khahArabic_dalArabic_thalArabic_raArabic_zainArabic_seenArabic_sheenArabic_sadArabic_dadArabic_tahArabic_zahArabic_ainArabic_ghainArabic_tatweelArabic_fehArabic_qafArabic_kafArabic_lamArabic_meemArabic_noonArabic_haArabic_hehArabic_wawArabic_alefmaksuraArabic_yehArabic_fathatanArabic_dammatanArabic_kasratanArabic_fathaArabic_dammaArabic_kasraArabic_shaddaArabic_sukunSerbian_djeMacedonia_gjeCyrillic_ioUkrainian_ieUkranian_jeMacedonia_dseUkrainian_iUkranian_iUkrainian_yiUkranian_yiCyrillic_jeSerbian_jeCyrillic_ljeSerbian_ljeCyrillic_njeSerbian_njeSerbian_tsheMacedonia_kjeUkrainian_ghe_with_upturnByelorussian_shortuCyrillic_dzheSerbian_dzenumerosignSerbian_DJEMacedonia_GJECyrillic_IOUkrainian_IEUkranian_JEMacedonia_DSEUkrainian_IUkranian_IUkrainian_YIUkranian_YICyrillic_JESerbian_JECyrillic_LJESerbian_LJECyrillic_NJESerbian_NJESerbian_TSHEMacedonia_KJEUkrainian_GHE_WITH_UPTURNByelorussian_SHORTUCyrillic_DZHESerbian_DZECyrillic_yuCyrillic_aCyrillic_beCyrillic_tseCyrillic_deCyrillic_ieCyrillic_efCyrillic_gheCyrillic_haCyrillic_iCyrillic_shortiCyrillic_kaCyrillic_elCyrillic_emCyrillic_enCyrillic_oCyrillic_peCyrillic_yaCyrillic_erCyrillic_esCyrillic_teCyrillic_uCyrillic_zheCyrillic_veCyrillic_softsignCyrillic_yeruCyrillic_zeCyrillic_shaCyrillic_eCyrillic_shchaCyrillic_cheCyrillic_hardsignCyrillic_YUCyrillic_ACyrillic_BECyrillic_TSECyrillic_DECyrillic_IECyrillic_EFCyrillic_GHECyrillic_HACyrillic_ICyrillic_SHORTICyrillic_KACyrillic_ELCyrillic_EMCyrillic_ENCyrillic_OCyrillic_PECyrillic_YACyrillic_ERCyrillic_ESCyrillic_TECyrillic_UCyrillic_ZHECyrillic_VECyrillic_SOFTSIGNCyrillic_YERUCyrillic_ZECyrillic_SHACyrillic_ECyrillic_SHCHACyrillic_CHECyrillic_HARDSIGNGreek_ALPHAaccentGreek_EPSILONaccentGreek_ETAaccentGreek_IOTAaccentGreek_IOTAdieresisGreek_IOTAdiaeresisGreek_OMICRONaccentGreek_UPSILONaccentGreek_UPSILONdieresisGreek_OMEGAaccentGreek_accentdieresisGreek_horizbarGreek_alphaaccentGreek_epsilonaccentGreek_etaaccentGreek_iotaaccentGreek_iotadieresisGreek_iotaaccentdieresisGreek_omicronaccentGreek_upsilonaccentGreek_upsilondieresisGreek_upsilonaccentdieresisGreek_omegaaccentGreek_ALPHAGreek_BETAGreek_GAMMAGreek_DELTAGreek_EPSILONGreek_ZETAGreek_ETAGreek_THETAGreek_IOTAGreek_KAPPAGreek_LAMDAGreek_LAMBDAGreek_MUGreek_NUGreek_XIGreek_OMICRONGreek_PIGreek_RHOGreek_SIGMAGreek_TAUGreek_UPSILONGreek_PHIGreek_CHIGreek_PSIGreek_OMEGAGreek_alphaGreek_betaGreek_gammaGreek_deltaGreek_epsilonGreek_zetaGreek_etaGreek_thetaGreek_iotaGreek_kappaGreek_lamdaGreek_lambdaGreek_muGreek_nuGreek_xiGreek_omicronGreek_piGreek_rhoGreek_sigmaGreek_finalsmallsigmaGreek_tauGreek_upsilonGreek_phiGreek_chiGreek_psiGreek_omegaleftradicaltopleftradicalhorizconnectortopintegralbotintegralvertconnectortopleftsqbracketbotleftsqbrackettoprightsqbracketbotrightsqbrackettopleftparensbotleftparenstoprightparensbotrightparensleftmiddlecurlybracerightmiddlecurlybracetopleftsummationbotleftsummationtopvertsummationconnectorbotvertsummationconnectortoprightsummationbotrightsummationrightmiddlesummationlessthanequalnotequalgreaterthanequalintegralthereforevariationinfinitynablaapproximatesimilarequalifonlyifimpliesidenticalradicalincludedinincludesintersectionunionlogicalandlogicalorpartialderivativefunctionleftarrowuparrowrightarrowdownarrowblanksoliddiamondcheckerboardhtffcrlfnlvtlowrightcorneruprightcornerupleftcornerlowleftcornercrossinglineshorizlinescan1horizlinescan3horizlinescan5horizlinescan7horizlinescan9lefttrighttbotttoptvertbaremspaceenspaceem3spaceem4spacedigitspacepunctspacethinspacehairspaceemdashendashsignifblankellipsisdoubbaselinedotonethirdtwothirdsonefifthtwofifthsthreefifthsfourfifthsonesixthfivesixthscareoffigdashleftanglebracketdecimalpointrightanglebracketmarkeroneeighththreeeighthsfiveeighthsseveneighthstrademarksignaturemarktrademarkincircleleftopentrianglerightopentriangleemopencircleemopenrectangleleftsinglequotemarkrightsinglequotemarkleftdoublequotemarkrightdoublequotemarkprescriptionpermilleminutessecondslatincrosshexagramfilledrectbulletfilledlefttribulletfilledrighttribulletemfilledcircleemfilledrectenopencircbulletenopensquarebulletopenrectbulletopentribulletupopentribulletdownopenstarenfilledcircbulletenfilledsqbulletfilledtribulletupfilledtribulletdownleftpointerrightpointerclubdiamondheartmaltesecrossdaggerdoubledaggercheckmarkballotcrossmusicalsharpmusicalflatmalesymbolfemalesymboltelephonetelephonerecorderphonographcopyrightcaretsinglelowquotemarkdoublelowquotemarkcursorleftcaretrightcaretdowncaretupcaretoverbardowntackupshoedownstileunderbarjotquaduptackcircleupstiledownshoerightshoeleftshoelefttackrighttackhebrew_doublelowlinehebrew_alephhebrew_bethebrew_bethhebrew_gimelhebrew_gimmelhebrew_dalethebrew_dalethhebrew_hehebrew_wawhebrew_zainhebrew_zayinhebrew_chethebrew_hethebrew_tethebrew_tethhebrew_yodhebrew_finalkaphhebrew_kaphhebrew_lamedhebrew_finalmemhebrew_memhebrew_finalnunhebrew_nunhebrew_samechhebrew_samekhhebrew_ayinhebrew_finalpehebrew_pehebrew_finalzadehebrew_finalzadihebrew_zadehebrew_zadihebrew_qophhebrew_kufhebrew_reshhebrew_shinhebrew_tawhebrew_tafThai_kokaiThai_khokhaiThai_khokhuatThai_khokhwaiThai_khokhonThai_khorakhangThai_ngonguThai_chochanThai_chochingThai_chochangThai_sosoThai_chochoeThai_yoyingThai_dochadaThai_topatakThai_thothanThai_thonangmonthoThai_thophuthaoThai_nonenThai_dodekThai_totaoThai_thothungThai_thothahanThai_thothongThai_nonuThai_bobaimaiThai_poplaThai_phophungThai_fofaThai_phophanThai_fofanThai_phosamphaoThai_momaThai_yoyakThai_roruaThai_ruThai_lolingThai_luThai_wowaenThai_sosalaThai_sorusiThai_sosuaThai_hohipThai_lochulaThai_oangThai_honokhukThai_paiyannoiThai_saraaThai_maihanakatThai_saraaaThai_saraamThai_saraiThai_saraiiThai_saraueThai_saraueeThai_sarauThai_sarauuThai_phinthuThai_maihanakat_maithoThai_bahtThai_saraeThai_saraaeThai_saraoThai_saraaimaimuanThai_saraaimaimalaiThai_lakkhangyaoThai_maiyamokThai_maitaikhuThai_maiekThai_maithoThai_maitriThai_maichattawaThai_thanthakhatThai_nikhahitThai_leksunThai_leknungThai_leksongThai_leksamThai_leksiThai_lekhaThai_lekhokThai_lekchetThai_lekpaetThai_lekkaoHangul_KiyeogHangul_SsangKiyeogHangul_KiyeogSiosHangul_NieunHangul_NieunJieujHangul_NieunHieuhHangul_DikeudHangul_SsangDikeudHangul_RieulHangul_RieulKiyeogHangul_RieulMieumHangul_RieulPieubHangul_RieulSiosHangul_RieulTieutHangul_RieulPhieufHangul_RieulHieuhHangul_MieumHangul_PieubHangul_SsangPieubHangul_PieubSiosHangul_SiosHangul_SsangSiosHangul_IeungHangul_JieujHangul_SsangJieujHangul_CieucHangul_KhieuqHangul_TieutHangul_PhieufHangul_HieuhHangul_AHangul_AEHangul_YAHangul_YAEHangul_EOHangul_EHangul_YEOHangul_YEHangul_OHangul_WAHangul_WAEHangul_OEHangul_YOHangul_UHangul_WEOHangul_WEHangul_WIHangul_YUHangul_EUHangul_YIHangul_IHangul_J_KiyeogHangul_J_SsangKiyeogHangul_J_KiyeogSiosHangul_J_NieunHangul_J_NieunJieujHangul_J_NieunHieuhHangul_J_DikeudHangul_J_RieulHangul_J_RieulKiyeogHangul_J_RieulMieumHangul_J_RieulPieubHangul_J_RieulSiosHangul_J_RieulTieutHangul_J_RieulPhieufHangul_J_RieulHieuhHangul_J_MieumHangul_J_PieubHangul_J_PieubSiosHangul_J_SiosHangul_J_SsangSiosHangul_J_IeungHangul_J_JieujHangul_J_CieucHangul_J_KhieuqHangul_J_TieutHangul_J_PhieufHangul_J_HieuhHangul_RieulYeorinHieuhHangul_SunkyeongeumMieumHangul_SunkyeongeumPieubHangul_PanSiosHangul_KkogjiDalrinIeungHangul_SunkyeongeumPhieufHangul_YeorinHieuhHangul_AraeAHangul_AraeAEHangul_J_PanSiosHangul_J_KkogjiDalrinIeungHangul_J_YeorinHieuhKorean_WonOEoeYdiaeresisEuroSign3270_Duplicate3270_FieldMark3270_Right23270_Left23270_BackTab3270_EraseEOF3270_EraseInput3270_Reset3270_Quit3270_PA13270_PA23270_PA33270_Test3270_Attn3270_CursorBlink3270_AltCursor3270_KeyClick3270_Jump3270_Ident3270_Rule3270_Copy3270_Play3270_Setup3270_Record3270_ChangeScreen3270_DeleteWord3270_ExSelect3270_CursorSelect3270_PrintScreen3270_EnterISO_LockISO_Level2_LatchISO_Level3_ShiftISO_Level3_LatchISO_Level3_LockISO_Group_LatchISO_Group_LockISO_Next_GroupISO_Next_Group_LockISO_Prev_GroupISO_Prev_Group_LockISO_First_GroupISO_First_Group_LockISO_Last_GroupISO_Last_Group_LockISO_Level5_ShiftISO_Level5_LatchISO_Level5_LockISO_Left_TabISO_Move_Line_UpISO_Move_Line_DownISO_Partial_Line_UpISO_Partial_Line_DownISO_Partial_Space_LeftISO_Partial_Space_RightISO_Set_Margin_LeftISO_Set_Margin_RightISO_Release_Margin_LeftISO_Release_Margin_RightISO_Release_Both_MarginsISO_Fast_Cursor_LeftISO_Fast_Cursor_RightISO_Fast_Cursor_UpISO_Fast_Cursor_DownISO_Continuous_UnderlineISO_Discontinuous_UnderlineISO_EmphasizeISO_Center_ObjectISO_Enterdead_gravedead_acutedead_circumflexdead_tildedead_perispomenidead_macrondead_brevedead_abovedotdead_diaeresisdead_aboveringdead_doubleacutedead_carondead_cedilladead_ogonekdead_iotadead_voiced_sounddead_semivoiced_sounddead_belowdotdead_hookdead_horndead_strokedead_abovecommadead_psilidead_abovereversedcommadead_dasiadead_doublegravedead_belowringdead_belowmacrondead_belowcircumflexdead_belowtildedead_belowbrevedead_belowdiaeresisdead_invertedbrevedead_belowcommadead_currencyAccessX_EnableAccessX_Feedback_EnableRepeatKeys_EnableSlowKeys_EnableBounceKeys_EnableStickyKeys_EnableMouseKeys_EnableMouseKeys_Accel_EnableOverlay1_EnableOverlay2_EnableAudibleBell_Enabledead_adead_Adead_edead_Edead_idead_Idead_odead_Odead_udead_Udead_small_schwadead_schwadead_capital_schwadead_SCHWAdead_greekdead_hamzadead_lowlinedead_aboveverticallinedead_belowverticallinedead_longsolidusoverlaychChCHc_hC_hC_HFirst_Virtual_ScreenPrev_Virtual_ScreenNext_Virtual_ScreenLast_Virtual_ScreenTerminate_ServerPointer_LeftPointer_RightPointer_UpPointer_DownPointer_UpLeftPointer_UpRightPointer_DownLeftPointer_DownRightPointer_Button_DfltPointer_Button1Pointer_Button2Pointer_Button3Pointer_Button4Pointer_Button5Pointer_DblClick_DfltPointer_DblClick1Pointer_DblClick2Pointer_DblClick3Pointer_DblClick4Pointer_DblClick5Pointer_Drag_DfltPointer_Drag1Pointer_Drag2Pointer_Drag3Pointer_Drag4Pointer_EnableKeysPointer_AcceleratePointer_DfltBtnNextPointer_DfltBtnPrevPointer_Drag5BackSpaceTabLinefeedClearReturnPauseScroll_LockSys_ReqEscapeMulti_keySunComposeKanjiMuhenkanHenkan_ModeHenkanRomajiHiraganaKatakanaHiragana_KatakanaZenkakuHankakuZenkaku_HankakuTourokuMassyoKana_LockKana_ShiftEisu_ShiftEisu_toggleHangulHangul_StartHangul_EndHangul_HanjaHangul_JamoHangul_RomajaCodeinputKanji_BangouHangul_CodeinputHangul_JeonjaHangul_BanjaHangul_PreHanjaHangul_PostHanjaSingleCandidateHangul_SingleCandidateMultipleCandidateZen_KohoHangul_MultipleCandidatePreviousCandidateMae_KohoHangul_PreviousCandidateHangul_SpecialHomeLeftUpRightDownPriorPage_UpSunPageUpNextPage_DownSunPageDownEndBeginSelectPrintSunPrint_ScreenExecuteInsertUndoSunUndoRedoSunAgainMenuFindSunFindCancelSunStopHelpBreakMode_switchscript_switchISO_Group_Shiftkana_switchArabic_switchGreek_switchHebrew_switchHangul_switchSunAltGraphNum_LockKP_SpaceKP_TabKP_EnterKP_F1KP_F2KP_F3KP_F4KP_HomeKP_LeftKP_UpKP_RightKP_DownKP_PriorKP_Page_UpKP_NextKP_Page_DownKP_EndKP_BeginKP_InsertKP_DeleteKP_MultiplyKP_AddKP_SeparatorKP_SubtractKP_DecimalKP_DivideKP_0KP_1KP_2KP_3KP_4KP_5KP_6KP_7KP_8KP_9KP_EqualF1F2F3F4F5F6F7F8F9F10F11L1F12L2F13L3F14L4F15L5F16L6F17L7F18L8F19L9F20L10F21R1F22R2F23R3F24R4F25R5F26R6F27R7F28R8F29R9F30R10F31R11F32R12F33R13F34R14F35R15Shift_LShift_RControl_LControl_RCaps_LockShift_LockMeta_LMeta_RAlt_LAlt_RSuper_LSuper_RHyper_LHyper_Rbraille_dot_1braille_dot_2braille_dot_3braille_dot_4braille_dot_5braille_dot_6braille_dot_7braille_dot_8braille_dot_9braille_dot_10DeleteVoidSymbolIbreveibreveWcircumflexwcircumflexYcircumflexycircumflexSCHWAObarredOhornohornUhornuhornZstrokezstrokeEZHOcaronocaronGcarongcaronschwaobarredezhcombining_gravecombining_acutecombining_tildecombining_hookcombining_belowdotCyrillic_GHE_barCyrillic_ghe_barCyrillic_ZHE_descenderCyrillic_zhe_descenderCyrillic_KA_descenderCyrillic_ka_descenderCyrillic_KA_vertstrokeCyrillic_ka_vertstrokeCyrillic_EN_descenderCyrillic_en_descenderCyrillic_U_straightCyrillic_u_straightCyrillic_U_straight_barCyrillic_u_straight_barCyrillic_HA_descenderCyrillic_ha_descenderCyrillic_CHE_descenderCyrillic_che_descenderCyrillic_CHE_vertstrokeCyrillic_che_vertstrokeCyrillic_SHHACyrillic_shhaCyrillic_SCHWACyrillic_schwaCyrillic_I_macronCyrillic_i_macronCyrillic_O_barCyrillic_o_barCyrillic_U_macronCyrillic_u_macronArmenian_AYBArmenian_BENArmenian_GIMArmenian_DAArmenian_YECHArmenian_ZAArmenian_EArmenian_ATArmenian_TOArmenian_ZHEArmenian_INIArmenian_LYUNArmenian_KHEArmenian_TSAArmenian_KENArmenian_HOArmenian_DZAArmenian_GHATArmenian_TCHEArmenian_MENArmenian_HIArmenian_NUArmenian_SHAArmenian_VOArmenian_CHAArmenian_PEArmenian_JEArmenian_RAArmenian_SEArmenian_VEVArmenian_TYUNArmenian_REArmenian_TSOArmenian_VYUNArmenian_PYURArmenian_KEArmenian_OArmenian_FEArmenian_apostropheArmenian_accentArmenian_sheshtArmenian_exclamArmenian_amanakArmenian_separation_markArmenian_butArmenian_questionArmenian_paruykArmenian_aybArmenian_benArmenian_gimArmenian_daArmenian_yechArmenian_zaArmenian_eArmenian_atArmenian_toArmenian_zheArmenian_iniArmenian_lyunArmenian_kheArmenian_tsaArmenian_kenArmenian_hoArmenian_dzaArmenian_ghatArmenian_tcheArmenian_menArmenian_hiArmenian_nuArmenian_shaArmenian_voArmenian_chaArmenian_peArmenian_jeArmenian_raArmenian_seArmenian_vevArmenian_tyunArmenian_reArmenian_tsoArmenian_vyunArmenian_pyurArmenian_keArmenian_oArmenian_feArmenian_ligature_ewArmenian_full_stopArmenian_verjaketArmenian_hyphenArmenian_yentamnaArabic_madda_aboveArabic_hamza_aboveArabic_hamza_belowArabic_0Arabic_1Arabic_2Arabic_3Arabic_4Arabic_5Arabic_6Arabic_7Arabic_8Arabic_9Arabic_percentArabic_superscript_alefArabic_ttehArabic_pehArabic_tchehArabic_ddalArabic_rrehArabic_jehArabic_vehArabic_kehehArabic_gafArabic_noon_ghunnaArabic_heh_doachashmeeArabic_heh_goalFarsi_yehArabic_farsi_yehArabic_yeh_bareeArabic_fullstopFarsi_0Farsi_1Farsi_2Farsi_3Farsi_4Farsi_5Farsi_6Farsi_7Farsi_8Farsi_9Sinh_ngSinh_h2Sinh_aSinh_aaSinh_aeSinh_aeeSinh_iSinh_iiSinh_uSinh_uuSinh_riSinh_riiSinh_luSinh_luuSinh_eSinh_eeSinh_aiSinh_oSinh_ooSinh_auSinh_kaSinh_khaSinh_gaSinh_ghaSinh_ng2Sinh_ngaSinh_caSinh_chaSinh_jaSinh_jhaSinh_nyaSinh_jnyaSinh_njaSinh_ttaSinh_tthaSinh_ddaSinh_ddhaSinh_nnaSinh_nddaSinh_thaSinh_thhaSinh_dhaSinh_dhhaSinh_naSinh_ndhaSinh_paSinh_phaSinh_baSinh_bhaSinh_maSinh_mbaSinh_yaSinh_raSinh_laSinh_vaSinh_shaSinh_sshaSinh_saSinh_haSinh_llaSinh_faSinh_alSinh_aa2Sinh_ae2Sinh_aee2Sinh_i2Sinh_ii2Sinh_u2Sinh_uu2Sinh_ru2Sinh_e2Sinh_ee2Sinh_ai2Sinh_o2Sinh_oo2Sinh_au2Sinh_lu2Sinh_ruu2Sinh_luu2Sinh_kunddaliyaGeorgian_anGeorgian_banGeorgian_ganGeorgian_donGeorgian_enGeorgian_vinGeorgian_zenGeorgian_tanGeorgian_inGeorgian_kanGeorgian_lasGeorgian_manGeorgian_narGeorgian_onGeorgian_parGeorgian_zharGeorgian_raeGeorgian_sanGeorgian_tarGeorgian_unGeorgian_pharGeorgian_kharGeorgian_ghanGeorgian_qarGeorgian_shinGeorgian_chinGeorgian_canGeorgian_jilGeorgian_cilGeorgian_charGeorgian_xanGeorgian_jhanGeorgian_haeGeorgian_heGeorgian_hieGeorgian_weGeorgian_harGeorgian_hoeGeorgian_fiBabovedotbabovedotDabovedotdabovedotFabovedotfabovedotLbelowdotlbelowdotMabovedotmabovedotPabovedotpabovedotSabovedotsabovedotTabovedottabovedotWgravewgraveWacutewacuteWdiaeresiswdiaeresisXabovedotxabovedotAbelowdotabelowdotAhookahookAcircumflexacuteacircumflexacuteAcircumflexgraveacircumflexgraveAcircumflexhookacircumflexhookAcircumflextildeacircumflextildeAcircumflexbelowdotacircumflexbelowdotAbreveacuteabreveacuteAbrevegraveabrevegraveAbrevehookabrevehookAbrevetildeabrevetildeAbrevebelowdotabrevebelowdotEbelowdotebelowdotEhookehookEtildeetildeEcircumflexacuteecircumflexacuteEcircumflexgraveecircumflexgraveEcircumflexhookecircumflexhookEcircumflextildeecircumflextildeEcircumflexbelowdotecircumflexbelowdotIhookihookIbelowdotibelowdotObelowdotobelowdotOhookohookOcircumflexacuteocircumflexacuteOcircumflexgraveocircumflexgraveOcircumflexhookocircumflexhookOcircumflextildeocircumflextildeOcircumflexbelowdotocircumflexbelowdotOhornacuteohornacuteOhorngraveohorngraveOhornhookohornhookOhorntildeohorntildeOhornbelowdotohornbelowdotUbelowdotubelowdotUhookuhookUhornacuteuhornacuteUhorngraveuhorngraveUhornhookuhornhookUhorntildeuhorntildeUhornbelowdotuhornbelowdotYgraveygraveYbelowdotybelowdotYhookyhookYtildeytildezerosuperiorfoursuperiorfivesuperiorsixsuperiorsevensuperioreightsuperiorninesuperiorzerosubscriptonesubscripttwosubscriptthreesubscriptfoursubscriptfivesubscriptsixsubscriptsevensubscripteightsubscriptninesubscriptEcuSignColonSignCruzeiroSignFFrancSignLiraSignMillSignNairaSignPesetaSignRupeeSignWonSignNewSheqelSignDongSignpartdifferentialemptysetelementofnotelementofcontainsassquarerootcuberootfourthrootdintegraltintegralbecausenotapproxeqapproxeqnotidenticalstricteqbraille_blankbraille_dots_1braille_dots_2braille_dots_12braille_dots_3braille_dots_13braille_dots_23braille_dots_123braille_dots_4braille_dots_14braille_dots_24braille_dots_124braille_dots_34braille_dots_134braille_dots_234braille_dots_1234braille_dots_5braille_dots_15braille_dots_25braille_dots_125braille_dots_35braille_dots_135braille_dots_235braille_dots_1235braille_dots_45braille_dots_145braille_dots_245braille_dots_1245braille_dots_345braille_dots_1345braille_dots_2345braille_dots_12345braille_dots_6braille_dots_16braille_dots_26braille_dots_126braille_dots_36braille_dots_136braille_dots_236braille_dots_1236braille_dots_46braille_dots_146braille_dots_246braille_dots_1246braille_dots_346braille_dots_1346braille_dots_2346braille_dots_12346braille_dots_56braille_dots_156braille_dots_256braille_dots_1256braille_dots_356braille_dots_1356braille_dots_2356braille_dots_12356braille_dots_456braille_dots_1456braille_dots_2456braille_dots_12456braille_dots_3456braille_dots_13456braille_dots_23456braille_dots_123456braille_dots_7braille_dots_17braille_dots_27braille_dots_127braille_dots_37braille_dots_137braille_dots_237braille_dots_1237braille_dots_47braille_dots_147braille_dots_247braille_dots_1247braille_dots_347braille_dots_1347braille_dots_2347braille_dots_12347braille_dots_57braille_dots_157braille_dots_257braille_dots_1257braille_dots_357braille_dots_1357braille_dots_2357braille_dots_12357braille_dots_457braille_dots_1457braille_dots_2457braille_dots_12457braille_dots_3457braille_dots_13457braille_dots_23457braille_dots_123457braille_dots_67braille_dots_167braille_dots_267braille_dots_1267braille_dots_367braille_dots_1367braille_dots_2367braille_dots_12367braille_dots_467braille_dots_1467braille_dots_2467braille_dots_12467braille_dots_3467braille_dots_13467braille_dots_23467braille_dots_123467braille_dots_567braille_dots_1567braille_dots_2567braille_dots_12567braille_dots_3567braille_dots_13567braille_dots_23567braille_dots_123567braille_dots_4567braille_dots_14567braille_dots_24567braille_dots_124567braille_dots_34567braille_dots_134567braille_dots_234567braille_dots_1234567braille_dots_8braille_dots_18braille_dots_28braille_dots_128braille_dots_38braille_dots_138braille_dots_238braille_dots_1238braille_dots_48braille_dots_148braille_dots_248braille_dots_1248braille_dots_348braille_dots_1348braille_dots_2348braille_dots_12348braille_dots_58braille_dots_158braille_dots_258braille_dots_1258braille_dots_358braille_dots_1358braille_dots_2358braille_dots_12358braille_dots_458braille_dots_1458braille_dots_2458braille_dots_12458braille_dots_3458braille_dots_13458braille_dots_23458braille_dots_123458braille_dots_68braille_dots_168braille_dots_268braille_dots_1268braille_dots_368braille_dots_1368braille_dots_2368braille_dots_12368braille_dots_468braille_dots_1468braille_dots_2468braille_dots_12468braille_dots_3468braille_dots_13468braille_dots_23468braille_dots_123468braille_dots_568braille_dots_1568braille_dots_2568braille_dots_12568braille_dots_3568braille_dots_13568braille_dots_23568braille_dots_123568braille_dots_4568braille_dots_14568braille_dots_24568braille_dots_124568braille_dots_34568braille_dots_134568braille_dots_234568braille_dots_1234568braille_dots_78braille_dots_178braille_dots_278braille_dots_1278braille_dots_378braille_dots_1378braille_dots_2378braille_dots_12378braille_dots_478braille_dots_1478braille_dots_2478braille_dots_12478braille_dots_3478braille_dots_13478braille_dots_23478braille_dots_123478braille_dots_578braille_dots_1578braille_dots_2578braille_dots_12578braille_dots_3578braille_dots_13578braille_dots_23578braille_dots_123578braille_dots_4578braille_dots_14578braille_dots_24578braille_dots_124578braille_dots_34578braille_dots_134578braille_dots_234578braille_dots_1234578braille_dots_678braille_dots_1678braille_dots_2678braille_dots_12678braille_dots_3678braille_dots_13678braille_dots_23678braille_dots_123678braille_dots_4678braille_dots_14678braille_dots_24678braille_dots_124678braille_dots_34678braille_dots_134678braille_dots_234678braille_dots_1234678braille_dots_5678braille_dots_15678braille_dots_25678braille_dots_125678braille_dots_35678braille_dots_135678braille_dots_235678braille_dots_1235678braille_dots_45678braille_dots_145678braille_dots_245678braille_dots_1245678braille_dots_345678braille_dots_1345678braille_dots_2345678braille_dots_12345678hpmute_acutemute_acutehpmute_gravemute_gravehpmute_asciicircummute_asciicircumhpmute_diaeresismute_diaeresishpmute_asciitildemute_asciitildehpliralirahpguilderguilderhpYdiaeresishpIOIOhplongminuslongminushpblockblockDdiaeresisDacute_accentDcedilla_accentDcircumflex_accentDgrave_accentDtildeDring_accentDRemovehpModelock1hpModelock2hpResetResethpSystemSystemhpUserUserhpClearLineClearLinehpInsertLineInsertLinehpDeleteLineDeleteLinehpInsertCharInsertCharhpDeleteCharDeleteCharhpBackTabBackTabhpKP_BackTabKP_BackTabExt16bit_LExt16bit_RosfCopyosfCutosfPasteosfBackTabosfBackSpaceosfClearosfEscapeosfAddModeosfPrimaryPasteosfQuickPasteosfPageLeftosfPageUposfPageDownosfPageRightosfActivateosfMenuBarosfLeftosfUposfRightosfDownosfEndLineosfBeginLineosfEndDataosfBeginDataosfPrevMenuosfNextMenuosfPrevFieldosfNextFieldosfSelectosfInsertosfUndoosfMenuosfCancelosfHelposfSelectAllosfDeselectAllosfReselectosfExtendosfRestoreosfDeleteSunFA_GraveSunFA_CircumSunFA_TildeSunFA_AcuteSunFA_DiaeresisSunFA_CedillaSunF36SunF37SunSys_ReqSunPropsSunFrontSunCopySunOpenSunPasteSunCutSunPowerSwitchSunAudioLowerVolumeSunAudioMuteSunAudioRaiseVolumeSunVideoDegaussSunVideoLowerBrightnessSunVideoRaiseBrightnessSunPowerSwitchShiftXF86MediaPlayPauseXF86ExitXF86AudioBassBoostXF86SportXF86BrightnessAutoXF86MonBrightnessAutoXF86DisplayOffXF86OKXF86GoToXF86InfoXF86VendorLogoXF86MediaSelectProgramGuideXF86MediaSelectHomeXF86MediaLanguageMenuXF86MediaTitleMenuXF86AudioChannelModeXF86AspectRatioXF86MediaSelectPCXF86MediaSelectTVXF86MediaSelectCableXF86MediaSelectVCRXF86MediaSelectVCRPlusXF86MediaSelectSatelliteXF86MediaSelectTapeXF86MediaSelectRadioXF86MediaSelectTunerXF86MediaPlayerXF86MediaSelectTeletextXF86DVDXF86MediaSelectDVDXF86MediaSelectAuxiliaryXF86AudioXF86ChannelUpXF86ChannelDownXF86MediaPlaySlowXF86BreakXF86NumberEntryModeXF86VideoPhoneXF86ZoomResetXF86EditorXF86GraphicsEditorXF86PresentationXF86DatabaseXF86VoicemailXF86AddressbookXF86DisplayToggleXF86SpellCheckXF86ContextMenuXF86MediaRepeatXF8610ChannelsUpXF8610ChannelsDownXF86ImagesXF86NotificationCenterXF86PickupPhoneXF86HangupPhoneXF86LinkPhoneXF86FnXF86Fn_EscXF86Fn_F1XF86Fn_F2XF86Fn_F3XF86Fn_F4XF86Fn_F5XF86Fn_F6XF86Fn_F7XF86Fn_F8XF86Fn_F9XF86Fn_F10XF86Fn_F11XF86Fn_F12XF86Fn_1XF86Fn_2XF86Fn_DXF86Fn_EXF86Fn_FXF86Fn_SXF86Fn_BXF86FnRightShiftXF86Numeric0XF86Numeric1XF86Numeric2XF86Numeric3XF86Numeric4XF86Numeric5XF86Numeric6XF86Numeric7XF86Numeric8XF86Numeric9XF86NumericStarXF86NumericPoundXF86NumericAXF86NumericBXF86NumericCXF86NumericDXF86CameraFocusXF86WPSButtonXF86CameraZoomInXF86CameraZoomOutXF86CameraUpXF86CameraDownXF86CameraLeftXF86CameraRightXF86AttendantOnXF86AttendantOffXF86AttendantToggleXF86LightsToggleXF86ALSToggleXF86RefreshRateToggleXF86ButtonconfigXF86TaskmanagerXF86JournalXF86ControlPanelXF86AppSelectXF86ScreensaverXF86VoiceCommandXF86AssistantXF86EmojiPickerXF86DictateXF86CameraAccessEnableXF86CameraAccessDisableXF86CameraAccessToggleXF86AccessibilityXF86DoNotDisturbXF86BrightnessMinXF86BrightnessMaxXF86KbdInputAssistPrevXF86KbdInputAssistNextXF86KbdInputAssistPrevgroupXF86KbdInputAssistNextgroupXF86KbdInputAssistAcceptXF86KbdInputAssistCancelXF86RightUpXF86RightDownXF86LeftUpXF86LeftDownXF86RootMenuXF86MediaTopMenuXF86Numeric11XF86Numeric12XF86AudioDescXF863DModeXF86NextFavoriteXF86StopRecordXF86PauseRecordXF86VODXF86UnmuteXF86FastReverseXF86SlowReverseXF86DataXF86OnScreenKeyboardXF86PrivacyScreenToggleXF86SelectiveScreenshotXF86NextElementXF86PreviousElementXF86AutopilotEngageToggleXF86MarkWaypointXF86SosXF86NavChartXF86FishingChartXF86SingleRangeRadarXF86DualRangeRadarXF86RadarOverlayXF86TraditionalSonarXF86ClearvuSonarXF86SidevuSonarXF86NavInfoXF86Macro1XF86Macro2XF86Macro3XF86Macro4XF86Macro5XF86Macro6XF86Macro7XF86Macro8XF86Macro9XF86Macro10XF86Macro11XF86Macro12XF86Macro13XF86Macro14XF86Macro15XF86Macro16XF86Macro17XF86Macro18XF86Macro19XF86Macro20XF86Macro21XF86Macro22XF86Macro23XF86Macro24XF86Macro25XF86Macro26XF86Macro27XF86Macro28XF86Macro29XF86Macro30XF86MacroRecordStartXF86MacroRecordStopXF86MacroPresetCycleXF86MacroPreset1XF86MacroPreset2XF86MacroPreset3XF86KbdLcdMenu1XF86KbdLcdMenu2XF86KbdLcdMenu3XF86KbdLcdMenu4XF86KbdLcdMenu5XF86PerformanceModeXF86Switch_VT_1XF86Switch_VT_2XF86Switch_VT_3XF86Switch_VT_4XF86Switch_VT_5XF86Switch_VT_6XF86Switch_VT_7XF86Switch_VT_8XF86Switch_VT_9XF86Switch_VT_10XF86Switch_VT_11XF86Switch_VT_12XF86UngrabXF86ClearGrabXF86Next_VModeXF86Prev_VModeXF86LogWindowTreeXF86LogGrabInfoXF86ModeLockXF86MonBrightnessUpXF86MonBrightnessDownXF86KbdLightOnOffXF86KbdBrightnessUpXF86KbdBrightnessDownXF86MonBrightnessCycleXF86StandbyXF86AudioLowerVolumeXF86AudioMuteXF86AudioRaiseVolumeXF86AudioPlayXF86AudioStopXF86AudioPrevXF86AudioNextXF86HomePageXF86MailXF86StartXF86SearchXF86AudioRecordXF86CalculatorXF86MemoXF86ToDoListXF86CalendarXF86PowerDownXF86ContrastAdjustXF86RockerUpXF86RockerDownXF86RockerEnterXF86BackXF86ForwardXF86StopXF86RefreshXF86PowerOffXF86WakeUpXF86EjectXF86ScreenSaverXF86WWWXF86SleepXF86FavoritesXF86AudioPauseXF86AudioMediaXF86MyComputerXF86VendorHomeXF86LightBulbXF86ShopXF86HistoryXF86OpenURLXF86AddFavoriteXF86HotLinksXF86BrightnessAdjustXF86FinanceXF86CommunityXF86AudioRewindXF86BackForwardXF86Launch0XF86Launch1XF86Launch2XF86Launch3XF86Launch4XF86Launch5XF86Launch6XF86Launch7XF86Launch8XF86Launch9XF86LaunchAXF86LaunchBXF86LaunchCXF86LaunchDXF86LaunchEXF86LaunchFXF86ApplicationLeftXF86ApplicationRightXF86BookXF86CDXF86MediaSelectCDXF86CalculaterXF86ClearXF86CloseXF86CopyXF86CutXF86DisplayXF86DOSXF86DocumentsXF86ExcelXF86ExplorerXF86GameXF86GoXF86iTouchXF86LogOffXF86MarketXF86MeetingXF86MenuKBXF86MenuPBXF86MySitesXF86NewXF86NewsXF86OfficeHomeXF86OpenXF86OptionXF86PasteXF86PhoneXF86QXF86ReplyXF86ReloadXF86RotateWindowsXF86RotationPBXF86RotationKBXF86SaveXF86ScrollUpXF86ScrollDownXF86ScrollClickXF86SendXF86SpellXF86SplitScreenXF86SupportXF86TaskPaneXF86TerminalXF86ToolsXF86TravelXF86UserPBXF86User1KBXF86User2KBXF86VideoXF86WheelButtonXF86WordXF86XferXF86ZoomInXF86ZoomOutXF86AwayXF86MessengerXF86WebCamXF86MailForwardXF86PicturesXF86MusicXF86BatteryXF86BluetoothXF86WLANXF86UWBXF86AudioForwardXF86AudioRepeatXF86AudioRandomPlayXF86SubtitleXF86AudioCycleTrackXF86CycleAngleXF86FrameBackXF86FrameForwardXF86TimeXF86SelectXF86ViewXF86TopMenuXF86RedXF86GreenXF86YellowXF86BlueXF86SuspendXF86HibernateXF86TouchpadToggleXF86TouchpadOnXF86TouchpadOffXF86AudioMicMuteXF86KeyboardXF86WWANXF86RFKillXF86AudioPresetXF86RotationLockToggleXF86FullScreen";

pub(super) static KEYSYM_TO_IDX: PhfMap<u32, u16> = PhfMap {
    key: 10656210946825422025,
    disps: &[(0, 3), (1, 1529), (0, 10), (0, 58), (0, 288), (0, 7), (0, 1615), (0, 6), (0, 42), (0, 0), (0, 12), (0, 690), (0, 6), (0, 123), (0, 689), (0, 688), (0, 24), (0, 1732), (0, 4), (0, 9), (0, 7), (0, 13), (0, 30), (0, 2), (0, 4), (0, 0), (0, 6), (0, 3), (0, 21), (0, 172), (0, 109), (0, 12), (0, 63), (0, 111), (0, 44), (0, 154), (0, 2346), (0, 31), (0, 9), (0, 1568), (0, 3), (1, 1761), (0, 2021), (0, 747), (0, 490), (0, 420), (0, 402), (0, 5), (0, 6), (0, 9), (0, 68), (0, 26), (0, 3), (0, 241), (0, 100), (0, 385), (0, 4), (0, 254), (0, 12), (0, 569), (0, 284), (0, 962), (0, 205), (0, 8), (0, 118), (0, 236), (0, 646), (0, 55), (0, 19), (0, 74), (0, 6), (0, 241), (0, 592), (0, 243), (0, 76), (0, 96), (0, 3), (0, 0), (0, 2011), (0, 5), (0, 12), (3, 294), (0, 22), (0, 21), (0, 0), (0, 0), (0, 91), (0, 6), (0, 54), (0, 294), (0, 239), (0, 3), (0, 44), (0, 12), (0, 2347), (0, 14), (0, 236), (0, 185), (0, 1), (0, 56), (0, 23), (0, 0), (0, 2096), (0, 52), (0, 128), (0, 2), (0, 601), (0, 103), (0, 744), (0, 126), (0, 16), (0, 1450), (0, 351), (0, 1), (1, 447), (0, 6), (0, 12), (0, 12), (0, 0), (0, 10), (0, 19), (0, 5), (0, 12), (0, 3), (0, 111), (0, 76), (0, 0), (0, 144), (0, 97), (0, 115), (0, 80), (0, 10), (0, 0), (0, 1), (0, 1), (0, 11), (0, 0), (0, 37), (0, 38), (0, 12), (0, 126), (0, 0), (0, 1091), (0, 123), (0, 4), (0, 2), (0, 145), (0, 0), (0, 2315), (0, 2308), (0, 4), (1, 735), (0, 145), (0, 0), (0, 14), (0, 7), (0, 2), (0, 2453), (0, 1816), (0, 5), (9, 449), (0, 0), (0, 113), (0, 576), (0, 3), (0, 0), (0, 85), (0, 1), (0, 0), (0, 716), (0, 2037), (0, 263), (0, 68), (0, 1), (0, 0), (0, 731), (0, 163), (0, 339), (0, 72), (0, 2), (0, 13), (0, 9), (1, 375), (0, 596), (0, 2010), (0, 30), (0, 210), (0, 1), (0, 1155), (0, 1022), (0, 5), (0, 3), (0, 6), (0, 202), (0, 0), (0, 297), (0, 273), (0, 26), (0, 3), (0, 234), (0, 6), (0, 1987), (0, 8), (0, 416), (0, 642), (7, 73), (0, 5), (1, 721), (0, 264), (0, 105), (0, 1963), (0, 790), (0, 102), (5, 480), (0, 1121), (0, 676), (1, 363), (0, 16), (0, 297), (0, 6), (0, 6), (0, 150), (0, 10), (0, 1), (1, 743), (0, 83), (0, 85), (2, 405), (0, 8), (0, 85), (1, 882), (0, 149), (0, 254), (0, 238), (0, 57), (0, 148), (0, 221), (0, 98), (0, 145), (0, 23), (0, 53), (0, 5), (0, 173), (0, 673), (0, 703), (0, 10), (4, 79), (0, 27), (0, 1071), (0, 1855), (0, 3), (3, 1510), (0, 115), (0, 8), (0, 4), (0, 9), (0, 165), (0, 0), (0, 195), (5, 2082), (0, 11), (1, 583), (1, 1672), (0, 6), (0, 24), (2, 18), (0, 11), (0, 98), (0, 725), (0, 2), (0, 508), (0, 1), (0, 71), (0, 277), (0, 17), (0, 0), (0, 12), (0, 214), (0, 0), (3, 2339), (0, 832), (0, 224), (2, 135), (0, 81), (0, 136), (0, 39), (0, 5), (0, 238), (0, 117), (0, 4), (0, 267), (0, 222), (0, 0), (0, 144), (0, 6), (0, 12), (0, 9), (0, 9), (0, 3), (0, 5), (0, 39), (0, 21), (0, 23), (0, 14), (0, 355), (0, 3), (0, 15), (0, 272), (0, 133), (0, 88), (0, 2431), (1, 121), (0, 1950), (0, 264), (0, 193), (0, 8), (0, 535), (0, 32), (0, 2), (1, 635), (0, 665), (0, 14), (0, 2063), (0, 11), (0, 121), (0, 32), (0, 2), (0, 0), (0, 22), (0, 151), (0, 184), (0, 692), (0, 6), (0, 269), (0, 7), (0, 168), (4, 2435), (0, 13), (0, 1), (0, 245), (0, 1135), (0, 203), (1, 1285), (0, 220), (0, 9), (0, 1971), (0, 470), (0, 11), (0, 397), (0, 11), (0, 86), (7, 1562), (0, 56), (7, 2019), (8, 1681), (0, 190), (0, 158), (0, 364), (12, 2413), (0, 1), (0, 54), (0, 695), (0, 5), (0, 2215), (0, 1270), (0, 0), (0, 637), (3, 2225), (0, 6), (1, 962), (0, 33), (0, 4), (1, 535), (2, 1369), (0, 19), (3, 1523), (0, 69), (11, 1092), (4, 680), (0, 4), (3, 1976), (0, 400), (0, 540), (7, 2225), (0, 1452), (0, 610), (0, 157), (7, 633), (0, 0), (0, 784), (0, 29), (0, 1), (7, 641), (0, 1178), (0, 22), (1, 1237), (1, 1718), (0, 426), (13, 2123), (1, 1248), (3, 2332), (1, 945), (0, 1140), (0, 1200), (0, 543), (9, 5), (2, 224), (0, 268), (3, 1957), (0, 238), (0, 10), (0, 0), (3, 818), (0, 23), (35, 2211), (25, 2155), (1, 366), (0, 266), (15, 57), (0, 84), (0, 42), (0, 414), (13, 1644), (0, 100), (0, 11), (0, 0), (0, 0), (0, 1097), (0, 0), (0, 3), (20, 1063), (0, 1906), (1, 460), (5, 1014), (0, 1), (0, 651), (0, 106), (0, 2), (0, 169), (5, 1266), (0, 2), (0, 199), (0, 52), (1, 2307), (0, 126), (0, 41), (0, 1), (0, 2014), (0, 0), (0, 28), (0, 1287), (13, 759), (0, 519), (1, 1946), (0, 3), (3, 711), (0, 6), (0, 193), (0, 327), (0, 23), (0, 3), (0, 19), (0, 9), (0, 21), (0, 1), (0, 9), (0, 183), (0, 0), (1, 1466), (0, 1253), (0, 193), (0, 460), (0, 455), (0, 0), (0, 321), (0, 3), (0, 0), (0, 13), (1, 878), (0, 140), (0, 73), (0, 12), (0, 33), (0, 508), (0, 1525), (0, 28), (0, 0), (1, 204), (0, 7), (0, 4), (0, 1864), (0, 175), (36, 2153), (43, 2380), (0, 97), (0, 509), (2, 269), (0, 7), (0, 262), (2, 519), (0, 12), (1, 2460), (5, 464), (0, 3), (22, 2299), (0, 3), (0, 0), (2, 601), (0, 8), (0, 5), (81, 40), (0, 9)],
    map: &[2063, 2361, 1624, 930, 2514, 1610, 2468, 1389, 483, 1925, 1774, 1109, 1305, 2407, 995, 906, 1570, 103, 1063, 1989, 1484, 537, 2563, 412, 847, 1910, 1845, 2499, 1101, 1847, 984, 454, 1068, 1811, 2332, 1947, 394, 2576, 743, 2399, 962, 1883, 723, 2523, 2074, 2011, 969, 1091, 1359, 1796, 931, 395, 1297, 1868, 1932, 495, 2053, 1785, 974, 393, 1078, 2442, 1337, 2465, 1055, 2325, 570, 1593, 259, 218, 1996, 543, 1107, 1960, 261, 560, 1288, 2539, 1214, 2230, 1922, 599, 1670, 305, 463, 690, 1075, 533, 996, 744, 1986, 453, 1203, 1064, 1911, 2243, 2139, 613, 127, 429, 473, 1217, 1085, 275, 981, 1165, 811, 1775, 1163, 1944, 1401, 353, 2321, 1245, 363, 2627, 1255, 1797, 51, 1560, 1564, 1760, 2076, 1195, 917, 1377, 1869, 2009, 1290, 2025, 1661, 1761, 565, 405, 1974, 540, 2322, 490, 2584, 583, 777, 1705, 135, 1150, 307, 877, 2064, 408, 1918, 2169, 561, 614, 1215, 2423, 501, 1923, 582, 1426, 286, 1554, 90, 567, 701, 487, 1505, 1708, 2484, 2102, 304, 1728, 2116, 1844, 2449, 835, 2282, 2134, 1199, 1541, 2, 1253, 359, 1479, 2525, 1489, 523, 360, 781, 1556, 2038, 1376, 1937, 2187, 2039, 2144, 2391, 2596, 123, 1131, 165, 757, 1693, 522, 312, 625, 1739, 2050, 349, 1303, 1699, 1420, 1782, 2340, 136, 2301, 2495, 1436, 37, 633, 820, 619, 2392, 2611, 1302, 1516, 809, 1108, 372, 1262, 1886, 1980, 2450, 118, 2350, 612, 2130, 1490, 1622, 689, 406, 605, 1623, 1538, 1475, 2291, 2077, 356, 2451, 787, 1422, 1658, 1607, 1689, 2188, 22, 2515, 2348, 1599, 1649, 246, 2019, 178, 1542, 2389, 2310, 117, 965, 720, 2490, 525, 12, 113, 1315, 730, 878, 2419, 84, 167, 1465, 1161, 344, 1880, 188, 471, 1786, 1618, 209, 1569, 1975, 2075, 901, 332, 836, 2597, 2266, 34, 1528, 1166, 802, 430, 1655, 23, 1935, 1487, 1402, 277, 2447, 1419, 403, 1870, 1351, 251, 27, 252, 885, 92, 7, 2582, 1659, 663, 2288, 1694, 783, 1025, 1701, 1476, 73, 2521, 2185, 617, 1596, 2173, 1583, 875, 1115, 2067, 484, 2309, 1841, 2024, 1398, 2114, 2226, 2587, 2294, 907, 2511, 538, 1630, 1103, 2213, 2189, 2268, 1466, 2171, 1162, 65, 668, 897, 2103, 2526, 1173, 1524, 833, 2500, 451, 2227, 1087, 2404, 215, 2167, 427, 2273, 1517, 2401, 886, 385, 2594, 2216, 1138, 2072, 2522, 1525, 1037, 1092, 2163, 1862, 1933, 2536, 2289, 887, 2182, 1997, 2345, 1052, 1846, 1982, 88, 2573, 971, 1325, 0, 1140, 2142, 1820, 826, 2248, 1584, 534, 876, 1076, 1987, 1174, 1842, 371, 1461, 1406, 2115, 1772, 2552, 298, 731, 535, 2110, 1148, 467, 272, 1099, 2569, 2561, 121, 1972, 1061, 1565, 1159, 2583, 1117, 13, 2507, 1248, 2036, 151, 1116, 642, 2534, 718, 1945, 2397, 894, 524, 1794, 1606, 1717, 496, 2352, 2165, 1866, 1249, 670, 1286, 1416, 636, 1930, 1089, 1994, 972, 1815, 2463, 1454, 1353, 721, 1333, 1919, 568, 2111, 810, 1053, 1768, 1387, 20, 1331, 1191, 2508, 465, 920, 2440, 2048, 2375, 531, 1984, 702, 558, 1507, 1773, 1920, 946, 2112, 563, 1872, 1771, 1254, 469, 104, 624, 1983, 728, 2330, 1233, 611, 331, 2318, 2497, 1062, 341, 1241, 208, 1894, 1193, 452, 1347, 1375, 1795, 1285, 299, 2058, 1958, 709, 1719, 1615, 1287, 764, 2061, 1446, 1629, 396, 1731, 1005, 2574, 1867, 712, 1266, 361, 2297, 2137, 1437, 1455, 1189, 2231, 2595, 133, 918, 1769, 1492, 276, 2408, 640, 1916, 700, 1152, 1357, 369, 1275, 1881, 283, 957, 482, 189, 1993, 2620, 1236, 943, 1058, 1855, 2300, 1921, 1200, 1539, 41, 559, 2376, 1250, 1759, 1176, 1088, 112, 78, 615, 1530, 179, 1242, 768, 2257, 552, 1194, 1620, 1710, 601, 679, 260, 710, 1744, 1036, 163, 1447, 982, 539, 214, 2037, 979, 1204, 1039, 2349, 1355, 2387, 2353, 1532, 797, 600, 329, 1680, 1514, 1006, 2383, 2443, 1009, 755, 1151, 1890, 2211, 839, 648, 2512, 1863, 1905, 816, 2441, 192, 2448, 834, 25, 370, 714, 2128, 1488, 186, 1973, 35, 1417, 664, 1536, 1656, 1022, 24, 1704, 616, 2380, 175, 1314, 1374, 724, 2008, 1473, 2174, 176, 780, 460, 2616, 641, 1178, 1597, 74, 244, 404, 1509, 2210, 2295, 630, 1692, 1463, 2269, 2186, 801, 342, 1853, 848, 1909, 10, 2323, 2051, 519, 1585, 2263, 945, 330, 1616, 106, 2433, 1442, 48, 8, 1186, 2237, 799, 1276, 2402, 649, 883, 21, 1526, 1400, 1034, 1529, 345, 2344, 2445, 2581, 280, 86, 1654, 2224, 643, 1657, 1369, 5, 2107, 970, 1023, 2472, 101, 895, 2003, 2519, 2206, 888, 1927, 1515, 873, 1688, 733, 691, 1594, 1522, 2628, 2281, 1677, 2417, 1787, 1141, 357, 11, 2292, 390, 2572, 1839, 2571, 1282, 1100, 1196, 2035, 109, 2462, 1561, 862, 1462, 968, 2398, 82, 1007, 1758, 1464, 232, 2264, 2548, 2287, 1030, 2481, 2086, 71, 1160, 1512, 1319, 1605, 687, 2405, 2200, 1035, 1718, 425, 2059, 311, 2159, 102, 1608, 187, 1310, 1586, 953, 204, 355, 77, 274, 105, 884, 719, 1033, 1559, 2520, 1149, 1474, 1995, 2509, 817, 1783, 1582, 2183, 2400, 2430, 793, 1985, 1840, 243, 606, 1936, 1770, 1020, 462, 1931, 2467, 440, 1581, 1059, 1146, 1050, 2559, 874, 60, 1371, 1906, 958, 729, 2113, 191, 1327, 2346, 536, 526, 916, 2549, 1349, 1157, 832, 2395, 340, 2308, 1086, 249, 1939, 532, 680, 1397, 174, 765, 688, 1132, 26, 638, 2256, 1283, 2056, 2049, 766, 1046, 2570, 2438, 2505, 1808, 1190, 2045, 1679, 954, 411, 779, 711, 2506, 2296, 756, 1329, 2109, 2161, 955, 2466, 759, 1902, 2030, 1864, 2214, 263, 181, 2431, 556, 1136, 2023, 271, 2341, 944, 1060, 892, 869, 1251, 592, 1017, 2094, 518, 1748, 1756, 428, 1971, 1198, 993, 1907, 1239, 1280, 598, 1443, 550, 660, 442, 1343, 1501, 604, 549, 1373, 2464, 1892, 1444, 63, 770, 707, 2624, 2022, 1929, 1956, 966, 2329, 1895, 164, 1729, 2386, 481, 2012, 1187, 1555, 580, 812, 2510, 1687, 2483, 1727, 1778, 1425, 1762, 699, 1992, 488, 1690, 771, 383, 1903, 1604, 2100, 2225, 557, 194, 119, 893, 1155, 1712, 941, 401, 172, 147, 546, 303, 1212, 2267, 180, 1230, 62, 190, 859, 778, 529, 554, 551, 1757, 2101, 18, 2065, 1767, 458, 2147, 177, 1860, 1981, 1240, 1673, 741, 1090, 1702, 1278, 1766, 1445, 753, 929, 347, 343, 122, 315, 2001, 2079, 1051, 2575, 775, 148, 647, 1978, 327, 310, 1678, 49, 1851, 610, 2603, 358, 1938, 528, 1706, 2098, 609, 386, 184, 1410, 661, 717, 1537, 742, 673, 2362, 767, 2577, 2198, 306, 392, 384, 1471, 1780, 1590, 2488, 350, 1613, 2626, 1494, 282, 173, 1685, 1300, 762, 1361, 1534, 1959, 1460, 1057, 791, 207, 339, 1647, 1523, 980, 1828, 2388, 1674, 156, 1865, 2293, 420, 1752, 239, 2208, 162, 588, 2416, 1781, 914, 685, 794, 1595, 1405, 1917, 46, 1008, 2124, 279, 1745, 795, 2222, 2261, 1675, 1031, 1388, 6, 301, 2252, 2562, 776, 1602, 19, 212, 1497, 426, 448, 1226, 1652, 2360, 639, 2579, 1322, 2253, 2609, 1709, 1535, 3, 2270, 2040, 1904, 2625, 202, 870, 631, 2199, 2406, 139, 480, 444, 1143, 433, 2141, 1016, 1457, 1928, 713, 1837, 1097, 1743, 1579, 1726, 185, 2218, 2209, 1700, 421, 2555, 830, 237, 2560, 860, 1809, 2496, 658, 2126, 1552, 566, 1513, 100, 991, 587, 1098, 1082, 715, 2223, 231, 1510, 2020, 521, 61, 1147, 530, 1833, 2546, 2146, 32, 2606, 1133, 881, 2580, 1603, 145, 2313, 1028, 2381, 2057, 2262, 250, 295, 2518, 2439, 4, 1048, 2284, 867, 53, 2565, 2622, 508, 1966, 1572, 1914, 675, 56, 1134, 1580, 1849, 2428, 1185, 628, 951, 256, 69, 107, 459, 872, 1913, 666, 2060, 1838, 515, 2617, 1113, 1967, 1968, 2331, 2247, 2493, 1386, 2396, 1418, 87, 2031, 846, 596, 1558, 1567, 1083, 2393, 2021, 1281, 1834, 323, 1145, 994, 871, 1313, 2557, 763, 2084, 1684, 446, 1024, 2212, 1878, 2314, 1806, 2531, 1957, 2087, 1508, 2482, 749, 1312, 2359, 581, 2006, 83, 2424, 2566, 2106, 1627, 1915, 1168, 695, 1049, 1568, 1000, 754, 686, 2618, 607, 2429, 1472, 1964, 646, 1941, 1724, 1591, 1381, 270, 1764, 1753, 2489, 2319, 516, 942, 1114, 1213, 1969, 1074, 2347, 963, 129, 739, 1619, 1001, 439, 1192, 1301, 1504, 927, 1237, 1835, 2148, 2547, 1818, 1339, 1954, 520, 1125, 269, 1047, 2033, 705, 760, 964, 1686, 1817, 267, 293, 1228, 2286, 1320, 853, 1942, 1754, 1879, 1589, 2085, 578, 2434, 1220, 1130, 258, 1396, 912, 1308, 450, 2556, 2285, 1392, 1614, 1002, 657, 1707, 1432, 1979, 2437, 1965, 1363, 608, 1901, 952, 1725, 938, 697, 1171, 740, 1044, 1750, 2517, 1277, 351, 500, 751, 2339, 1648, 389, 597, 544, 2280, 1976, 494, 1893, 1158, 2254, 1238, 1640, 2120, 1073, 678, 1592, 1755, 1850, 1188, 317, 706, 1740, 294, 1819, 1765, 1018, 845, 937, 2018, 589, 1720, 2042, 2608, 696, 493, 158, 57, 1711, 2414, 1154, 1221, 1816, 1440, 1671, 1029, 325, 505, 456, 1714, 146, 423, 1527, 2092, 479, 47, 1557, 131, 324, 1653, 507, 1224, 297, 2240, 2122, 199, 510, 2371, 58, 790, 367, 2095, 2444, 1485, 1733, 2567, 1900, 949, 2034, 555, 1645, 2196, 437, 1713, 154, 1814, 2452, 141, 2498, 575, 1521, 182, 2382, 956, 157, 2238, 2492, 932, 1638, 238, 59, 1852, 2096, 320, 2315, 1021, 314, 2028, 1553, 418, 1683, 2384, 2073, 338, 334, 2203, 1628, 2317, 2259, 221, 2607, 644, 206, 2403, 693, 424, 1511, 2251, 326, 33, 1600, 1650, 140, 1533, 2592, 2184, 1807, 171, 1014, 391, 1210, 1433, 2179, 527, 1407, 1, 868, 168, 1486, 2145, 708, 2479, 2197, 2316, 337, 1323, 2044, 402, 1634, 1321, 1577, 1452, 2047, 1480, 336, 241, 2614, 879, 2338, 1518, 2255, 1912, 2180, 858, 1227, 750, 382, 40, 1639, 656, 2373, 2619, 2379, 431, 134, 547, 2494, 2368, 115, 2558, 676, 2356, 2260, 843, 1625, 1026, 1751, 2275, 594, 2390, 2604, 200, 2304, 904, 1080, 1676, 634, 2097, 1857, 2593, 622, 2221, 824, 409, 170, 2503, 683, 1601, 623, 865, 1015, 1045, 1587, 1506, 1651, 1179, 1121, 95, 1573, 2533, 1691, 2501, 2504, 2246, 75, 235, 1142, 913, 786, 1578, 2615, 2426, 2369, 935, 55, 432, 854, 457, 1003, 1831, 160, 419, 2082, 1858, 513, 1071, 2600, 882, 1499, 143, 659, 1456, 726, 928, 844, 502, 1950, 1056, 242, 1955, 236, 1341, 2605, 1164, 905, 1111, 2554, 1279, 2568, 1804, 503, 2545, 1566, 1891, 603, 2435, 1042, 2004, 1876, 2068, 1832, 1635, 2422, 2046, 2093, 1823, 2529, 579, 2014, 2307, 2365, 1393, 1180, 1309, 1943, 1722, 855, 698, 851, 752, 161, 2041, 857, 1861, 1562, 2026, 2491, 1898, 880, 1010, 1394, 541, 2337, 940, 671, 1122, 2532, 1385, 737, 1887, 436, 476, 30, 2370, 1800, 1299, 1646, 1962, 925, 1211, 388, 2150, 1424, 1367, 1441, 585, 514, 939, 1888, 1721, 2149, 1741, 492, 1493, 291, 1805, 662, 1153, 761, 978, 703, 2427, 989, 120, 2432, 285, 2363, 1952, 748, 309, 2204, 1453, 1306, 85, 684, 1218, 2054, 2544, 1081, 1079, 1669, 2276, 1723, 1926, 734, 278, 379, 1877, 1430, 203, 2157, 2242, 1790, 1810, 788, 936, 1899, 1812, 921, 1395, 2476, 545, 2066, 2333, 1459, 694, 1884, 1112, 368, 2377, 1294, 999, 66, 1264, 1801, 1793, 1836, 2564, 1295, 1438, 290, 1668, 926, 1208, 1738, 257, 785, 629, 727, 292, 1802, 704, 645, 674, 321, 814, 769, 1144, 2436, 1977, 1951, 738, 2358, 435, 1311, 1123, 506, 1128, 1219, 1520, 1763, 375, 815, 2234, 967, 2367, 2623, 2193, 1779, 1672, 308, 2117, 300, 571, 1715, 2283, 197, 1431, 591, 1963, 1495, 890, 2194, 977, 42, 807, 1428, 1209, 399, 1643, 1184, 1970, 1408, 1483, 1225, 2320, 287, 76, 201, 381, 2425, 2585, 1697, 152, 2043, 947, 72, 1439, 2461, 2032, 1365, 228, 52, 2328, 2132, 1636, 2071, 1990, 2305, 2016, 828, 2513, 318, 116, 1859, 416, 44, 2202, 651, 1626, 1169, 2342, 1004, 2471, 222, 1384, 1027, 590, 2155, 132, 1737, 2480, 1043, 376, 1662, 517, 229, 498, 99, 959, 2590, 29, 2105, 1012, 130, 1519, 198, 2455, 2236, 866, 2195, 1588, 400, 245, 620, 1644, 1345, 2108, 380, 2205, 1856, 1135, 378, 1118, 1666, 2192, 910, 1469, 821, 586, 2612, 1571, 626, 1632, 1698, 1041, 31, 413, 654, 915, 1637, 2302, 2090, 155, 2394, 348, 2601, 417, 856, 224, 38, 1181, 2312, 302, 902, 841, 1072, 1547, 652, 2201, 2477, 2235, 335, 2542, 1040, 732, 2541, 1095, 407, 2249, 219, 144, 891, 1129, 1663, 2069, 93, 1824, 1013, 681, 2005, 2366, 789, 2007, 2591, 1873, 397, 28, 211, 863, 248, 562, 621, 230, 489, 829, 1390, 1871, 410, 911, 1633, 477, 1682, 1124, 387, 861, 2274, 1470, 16, 1576, 2245, 1825, 233, 2118, 2271, 837, 1296, 1409, 1382, 2413, 899, 1730, 2070, 210, 504, 268, 2239, 511, 81, 1948, 216, 2343, 1411, 2326, 1548, 2017, 2543, 842, 169, 1096, 2207, 990, 2478, 2250, 975, 2232, 64, 898, 1126, 2357, 220, 2378, 1821, 632, 266, 1791, 68, 2081, 745, 2355, 1563, 2143, 948, 758, 213, 1991, 2151, 576, 96, 2540, 1776, 288, 1177, 1732, 735, 70, 849, 2409, 1065, 262, 2550, 474, 1378, 669, 2334, 852, 997, 97, 1415, 1257, 1612, 1105, 2002, 1383, 1391, 2474, 1070, 434, 2013, 838, 1734, 1949, 1885, 1550, 2272, 987, 2055, 2029, 2172, 1798, 1998, 1093, 922, 774, 1292, 2335, 1307, 2027, 1551, 976, 485, 1940, 1934, 2602, 572, 1434, 1667, 627, 672, 2502, 2217, 1069, 1110, 2613, 1450, 2538, 1822, 2421, 1077, 961, 1924, 1216, 1549, 2470, 2410, 986, 98, 1265, 2278, 573, 1792, 998, 1953, 1988, 1448, 736, 1205, 2052, 1897, 1379, 475, 2153, 316, 377, 1170, 1746, 983, 1259, 1423, 366, 1380, 1260, 1882, 1946, 919, 584, 1182, 2327, 722, 1316, 923, 1119, 1830, 1172, 153, 205, 1120, 1206, 682, 2598, 2475, 1784, 1293, 1896, 1961, 1019, 1716, 1777, 313, 569, 1829, 2080, 497, 1736, 1435, 2181, 512, 2277, 227, 773, 1889, 2010, 1304, 108, 933, 142, 574, 2015, 137, 1451, 509, 17, 499, 319, 2385, 1183, 2091, 1449, 1202, 1813, 125, 564, 2415, 1788, 289, 2089, 2279, 1284, 2119, 782, 1631, 1421, 362, 1874, 374, 1481, 1641, 1747, 1875, 114, 183, 1803, 491, 747, 1167, 1298, 822, 1695, 2178, 2336, 1429, 746, 1207, 149, 50, 1664, 985, 653, 725, 1749, 284, 1156, 1735, 138, 992, 414, 2303, 2553, 2412, 803, 2298, 1544, 934, 2446, 637, 950, 89, 2104, 2527, 2473, 1318, 792, 1403, 223, 1412, 542, 2078, 39, 2374, 124, 1609, 94, 2530, 296, 255, 677, 2588, 1477, 196, 1660, 2190, 2453, 1317, 2176, 2460, 2244, 2088, 1540, 14, 635, 593, 1032, 1399, 1482, 864, 2485, 2219, 655, 111, 2578, 2458, 819, 1696, 2099, 1826, 2299, 1414, 908, 1427, 328, 2177, 1467, 772, 618, 45, 2456, 1742, 36, 415, 813, 2351, 264, 234, 2516, 1642, 2420, 1503, 805, 2364, 2354, 650, 900, 924, 373, 1038, 1621, 2459, 1458, 54, 398, 2083, 364, 1575, 79, 1413, 43, 2062, 253, 1478, 2233, 665, 2589, 1404, 1127, 903, 2454, 960, 486, 2175, 195, 2487, 1703, 2469, 2528, 1175, 346, 2306, 2290, 1843, 80, 595, 2220, 15, 1611, 225, 692, 240, 1854, 1531, 553, 2215, 2551, 2486, 247, 1066, 2457, 1848, 1665, 273, 217, 1617, 2228, 1598, 1827, 1106, 2229, 1491, 226, 2411, 91, 1067, 2535, 896, 1011, 2621, 840, 1468, 909, 1546, 2372, 1094, 988, 1799, 1908, 2324, 1999, 2191, 850, 784, 2599, 973, 265, 2311, 2524, 254, 2258, 2000, 716, 1054, 889, 2586, 1789, 2610, 1335, 2418, 1681, 667],
    _phantom: core::marker::PhantomData,
};

pub(super) static NAME_TO_IDX: PhfMap<[u8], u16> = PhfMap {
    key: 10656210946825422025,
    disps: &[(0, 40), (0, 0), (0, 10), (0, 9), (0, 14), (0, 5), (0, 0), (0, 4), (0, 17), (0, 0), (0, 2), (0, 14), (0, 5), (0, 0), (0, 0), (0, 5), (0, 7), (0, 5), (0, 0), (0, 0), (0, 25), (0, 1), (0, 6), (0, 14), (0, 5), (0, 0), (0, 14), (0, 9), (0, 1), (0, 6), (0, 1), (0, 7), (0, 2), (0, 4), (0, 6), (0, 0), (0, 12), (0, 78), (0, 3), (0, 8), (0, 3), (0, 0), (0, 20), (0, 8), (0, 23), (0, 0), (0, 0), (0, 10), (0, 0), (0, 3), (0, 27), (0, 5), (0, 0), (0, 1), (0, 0), (0, 14), (0, 5), (0, 1), (0, 0), (0, 15), (0, 0), (0, 0), (0, 0), (0, 0), (0, 3), (0, 36), (0, 0), (0, 6), (0, 58), (0, 3), (0, 28), (0, 4), (0, 0), (0, 8), (0, 2), (0, 4), (0, 19), (0, 0), (0, 0), (0, 1), (0, 10), (0, 35), (0, 12), (0, 13), (0, 77), (0, 0), (0, 1), (0, 6), (0, 0), (0, 0), (0, 20), (0, 0), (0, 42), (0, 5), (0, 14), (0, 11), (0, 0), (0, 13), (0, 13), (0, 0), (0, 10), (0, 83), (0, 0), (0, 100), (0, 0), (0, 1), (0, 0), (0, 2), (0, 32), (0, 14), (0, 25), (0, 6), (0, 25), (0, 16), (0, 0), (0, 1), (0, 2), (0, 0), (0, 8), (0, 6), (0, 0), (0, 29), (0, 0), (0, 42), (0, 6), (0, 1), (0, 4), (0, 101), (0, 7), (0, 17), (0, 5), (0, 3), (0, 2), (0, 79), (0, 0), (0, 8), (0, 144), (0, 0), (0, 12), (0, 20), (0, 0), (0, 0), (0, 1), (0, 1), (0, 57), (0, 0), (0, 2), (0, 3), (0, 19), (0, 13), (0, 83), (0, 2), (0, 0), (0, 7), (0, 34), (0, 6), (0, 7), (0, 3), (0, 1), (0, 9), (0, 31), (0, 3), (0, 3), (0, 5), (0, 10), (0, 0), (0, 81), (0, 57), (0, 12), (0, 20), (0, 1), (0, 108), (0, 0), (0, 0), (0, 3), (0, 0), (0, 25), (0, 2), (0, 4), (0, 43), (0, 20), (0, 0), (0, 19), (0, 6), (0, 60), (0, 7), (0, 4), (0, 4), (0, 36), (0, 1), (0, 5), (0, 15), (0, 53), (0, 21), (0, 34), (0, 0), (0, 8), (0, 0), (0, 0), (0, 0), (0, 2), (0, 2), (0, 1), (0, 4), (0, 1), (0, 4), (0, 1), (0, 35), (0, 2), (0, 17), (0, 0), (0, 3), (0, 12), (0, 3), (0, 11), (0, 9), (0, 1), (0, 0), (0, 24), (0, 25), (0, 2), (0, 8), (0, 47), (0, 18), (0, 0), (0, 1), (0, 129), (0, 6), (0, 0), (0, 3), (0, 0), (0, 3), (0, 41), (0, 147), (0, 1), (0, 11), (0, 0), (0, 19), (0, 0), (0, 2), (0, 2), (0, 14), (0, 0), (0, 54), (0, 13), (0, 0), (0, 21), (0, 16), (0, 2), (0, 23), (0, 18), (0, 19), (0, 9), (0, 0), (0, 9), (0, 4), (0, 6), (0, 23), (0, 0), (0, 14), (0, 0), (0, 3), (0, 30), (0, 5), (0, 0), (0, 2), (0, 77), (0, 14), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 23), (0, 5), (0, 1), (0, 5), (0, 5), (0, 5), (0, 4), (0, 8), (0, 0), (0, 0), (0, 31), (0, 375), (0, 2), (0, 22), (0, 5), (0, 28), (0, 19), (0, 0), (0, 36), (0, 37), (0, 0), (0, 0), (0, 17), (0, 1), (0, 0), (0, 0), (0, 5), (0, 32), (0, 0), (0, 0), (0, 16), (0, 0), (0, 37), (0, 106), (0, 7), (0, 134), (0, 5), (0, 4), (0, 0), (0, 2), (0, 17), (0, 0), (0, 199), (0, 3), (0, 378), (0, 0), (0, 27), (0, 5), (0, 9), (0, 98), (0, 11), (0, 7), (0, 2), (0, 1), (0, 0), (0, 15), (0, 54), (0, 4), (0, 0), (0, 3), (0, 8), (0, 7), (0, 9), (0, 0), (0, 38), (0, 45), (0, 0), (0, 74), (0, 0), (0, 3), (0, 2), (0, 43), (0, 15), (0, 30), (0, 2), (0, 5), (0, 18), (0, 0), (0, 11), (0, 15), (0, 73), (0, 4), (0, 15), (0, 4), (0, 18), (0, 37), (0, 3), (0, 10), (0, 12), (0, 0), (0, 0), (0, 0), (0, 14), (0, 1), (0, 12), (0, 0), (0, 2), (0, 0), (0, 67), (0, 3), (0, 33), (0, 33), (0, 0), (0, 73), (0, 2), (0, 381), (0, 1), (0, 66), (0, 7), (0, 19), (0, 2), (0, 1), (0, 19), (0, 0), (0, 17), (0, 4), (0, 0), (0, 1), (0, 0), (0, 15), (0, 0), (0, 0), (0, 1), (0, 2), (0, 1), (0, 0), (0, 1), (0, 46), (0, 4), (0, 4), (0, 0), (0, 2), (0, 2), (0, 18), (0, 9), (0, 0), (0, 0), (0, 20), (0, 10), (0, 4), (0, 36), (0, 0), (0, 6), (0, 2), (0, 42), (0, 333), (0, 42), (0, 0), (0, 39), (0, 0), (0, 5), (0, 1), (0, 0), (0, 2), (0, 6), (0, 39), (0, 49), (0, 0), (0, 8), (0, 0), (0, 0), (0, 2), (0, 11), (0, 8), (0, 0), (0, 0), (0, 17), (0, 1), (0, 1), (0, 45), (0, 2), (0, 21), (0, 9), (0, 0), (0, 2), (0, 27), (0, 1), (0, 3), (0, 62), (0, 12), (0, 12), (0, 0), (0, 7), (0, 0), (0, 209), (0, 4), (0, 0), (0, 0), (0, 49), (0, 7), (0, 55), (0, 328), (0, 13), (0, 69), (0, 48), (0, 9), (0, 72), (0, 0), (0, 154), (0, 10), (0, 160), (0, 1), (0, 4), (0, 154), (0, 2), (0, 2), (0, 35), (0, 45), (0, 14), (0, 686), (0, 62), (0, 3), (0, 1), (0, 1), (0, 1), (0, 237), (0, 5), (0, 113), (0, 0), (0, 64), (0, 4), (0, 2), (0, 64), (0, 123), (0, 0), (0, 10), (0, 13), (0, 0), (0, 13), (0, 4), (0, 5), (0, 8), (0, 2), (0, 0), (0, 42), (0, 3), (0, 0), (0, 0), (0, 1), (0, 0), (0, 841), (0, 88), (0, 2), (0, 6), (0, 0), (0, 79), (0, 26), (0, 85), (0, 3), (0, 5), (0, 0), (0, 0), (0, 45), (0, 61), (0, 106), (0, 1), (0, 1), (0, 7), (0, 15), (0, 0), (0, 0), (0, 0), (0, 2), (0, 22), (0, 77), (0, 69), (0, 0), (0, 15), (0, 0), (0, 10), (0, 2), (0, 0), (0, 150), (0, 12), (0, 1), (0, 47), (0, 440), (0, 0), (0, 11), (0, 0), (0, 14), (0, 3), (0, 13), (0, 66), (0, 412), (0, 0), (0, 2), (0, 0), (0, 0), (0, 12), (0, 82), (0, 20), (0, 0), (0, 1), (0, 13), (0, 4), (0, 0), (0, 2), (0, 0), (0, 2), (0, 388), (0, 9), (0, 0), (0, 91), (0, 109), (0, 2), (0, 9), (0, 1), (0, 1), (0, 4), (0, 3), (0, 15), (0, 742), (0, 8), (0, 49), (0, 9), (0, 214), (0, 0), (0, 5), (0, 17), (0, 83), (0, 42), (0, 0), (0, 24), (0, 81), (0, 14), (0, 29), (0, 10), (0, 26), (0, 33), (0, 15), (0, 6), (0, 2), (0, 0), (0, 38), (0, 2), (0, 2), (0, 0), (0, 23), (0, 19), (0, 9), (0, 5), (0, 33), (0, 652), (0, 0), (0, 3), (0, 6), (0, 149), (0, 2), (0, 0), (0, 3), (0, 23), (0, 0), (0, 71), (0, 9), (0, 0), (0, 12), (0, 0), (0, 3), (0, 1), (0, 23), (0, 115), (0, 11), (0, 11), (0, 16), (0, 171), (0, 14), (0, 84), (0, 8), (0, 190), (0, 52), (0, 91), (0, 0), (0, 1), (0, 1), (0, 33), (0, 39), (0, 1), (0, 11), (0, 0), (0, 51), (0, 8), (0, 108), (0, 157), (0, 0), (0, 257), (0, 0), (0, 57), (0, 17), (0, 0), (0, 3), (0, 58), (0, 108), (0, 165), (0, 5), (0, 269), (0, 0), (0, 18), (0, 17), (0, 1), (0, 35), (0, 36), (0, 0), (0, 23), (0, 1), (0, 101), (0, 55), (0, 27), (0, 43), (0, 120), (0, 392), (0, 9), (0, 0), (0, 1), (0, 33), (0, 0), (0, 724), (0, 32), (0, 42), (0, 19), (0, 0), (0, 0), (0, 21), (0, 19), (0, 3), (0, 42), (0, 1), (0, 6), (0, 2), (0, 5), (0, 33), (0, 31), (0, 10), (0, 0), (0, 85), (0, 1), (0, 17), (0, 6), (0, 8), (0, 7), (0, 0), (0, 22), (0, 1), (0, 69), (0, 0), (0, 6), (0, 173), (0, 14), (0, 14), (0, 72), (0, 231), (0, 173), (0, 0), (0, 0), (0, 0), (0, 8), (0, 14), (0, 1), (0, 0), (0, 7), (0, 21), (0, 4), (0, 7), (0, 8), (0, 0), (0, 49), (0, 154), (0, 17), (0, 61), (0, 203), (0, 22), (0, 0), (0, 1), (0, 0), (0, 52), (0, 7), (0, 0), (0, 4), (0, 0), (0, 128), (0, 17), (0, 0), (0, 18), (0, 51), (0, 1), (0, 2), (0, 220), (0, 476), (0, 17), (0, 2), (0, 43), (0, 7), (0, 0), (0, 60), (0, 40), (0, 0), (0, 12), (0, 23), (0, 9), (0, 3), (0, 159), (0, 2), (0, 1), (0, 60), (0, 0), (0, 174), (0, 96), (0, 92), (0, 74), (0, 301), (0, 1), (0, 13), (0, 29), (0, 938), (0, 25), (0, 6), (0, 0), (0, 12), (0, 15), (0, 2), (0, 41), (0, 28), (0, 237), (0, 1), (0, 0), (0, 2), (0, 71), (0, 18), (0, 90), (0, 3), (0, 1), (0, 240), (0, 0), (0, 4), (0, 21), (0, 51), (0, 5), (0, 6), (0, 2), (0, 19), (0, 851), (0, 52), (0, 141), (0, 99), (0, 16), (0, 21), (0, 0), (0, 11), (0, 0), (0, 49), (0, 11), (0, 49), (0, 10), (0, 36), (0, 1012), (0, 1), (0, 322), (0, 3), (0, 136), (0, 67), (0, 402), (0, 1), (0, 0), (0, 7), (0, 0), (0, 217), (0, 75), (0, 17), (0, 13), (0, 61), (0, 0), (0, 27), (0, 30), (0, 67), (0, 53), (0, 265), (0, 206), (0, 7), (0, 187), (0, 32), (0, 14), (0, 109), (0, 0), (0, 319), (0, 173), (0, 1479), (0, 0), (0, 188), (0, 2), (0, 468), (0, 14), (0, 44), (0, 1509), (0, 0), (0, 19), (0, 34), (0, 46), (0, 0), (0, 0), (0, 14), (0, 43), (0, 54), (0, 0), (0, 7), (0, 104), (0, 1114), (0, 0), (0, 0), (0, 40), (0, 6), (0, 5), (0, 78), (0, 14), (0, 35), (0, 92), (0, 2), (0, 0), (0, 24), (0, 0), (0, 263), (0, 0), (0, 5), (0, 179), (0, 4), (0, 28), (0, 41), (0, 94), (0, 0), (0, 163), (0, 34), (0, 338), (0, 343), (0, 54), (0, 110), (0, 7), (0, 0), (0, 112), (0, 39), (0, 4), (0, 61), (0, 49), (0, 9), (0, 33), (0, 150), (0, 21), (0, 8), (0, 2), (0, 1), (0, 1681), (0, 42), (0, 815), (0, 170), (0, 53), (0, 2), (0, 3), (0, 38), (0, 92), (0, 80), (0, 21), (0, 155), (0, 44), (0, 24), (0, 0), (0, 0), (0, 58), (0, 0), (0, 0), (0, 6), (0, 1544), (0, 44), (0, 63), (0, 3), (0, 45), (0, 11), (0, 0), (0, 821), (0, 2), (0, 744), (0, 0), (0, 36), (0, 0), (0, 104), (0, 1398), (0, 228), (0, 9), (0, 1915), (0, 17), (0, 633), (0, 440), (0, 0), (0, 1), (0, 1797), (0, 0), (0, 30), (0, 234), (0, 7), (0, 444), (0, 12), (0, 0), (0, 0), (0, 112), (0, 637), (0, 0), (0, 197), (0, 100), (0, 33), (0, 225), (0, 434), (0, 0), (0, 280), (0, 11), (0, 171), (0, 51), (0, 42), (0, 3), (0, 15), (0, 59), (0, 532), (0, 81), (0, 2), (0, 3), (0, 2137), (0, 1), (0, 3), (0, 3), (0, 0), (0, 118), (0, 86), (0, 2), (0, 26), (0, 54), (0, 1), (0, 150), (0, 121), (0, 16), (0, 150), (0, 4), (0, 51), (0, 17), (0, 291), (0, 15), (0, 49), (0, 93), (0, 4), (0, 158), (0, 244), (0, 214), (0, 1379), (0, 213), (0, 164), (0, 45)],
    map: &[2165, 177, 28, 1391, 2146, 869, 2337, 2268, 2587, 1731, 1224, 1088, 772, 2598, 1384, 859, 862, 100, 2032, 1046, 979, 2486, 326, 1687, 1456, 2519, 453, 1960, 492, 2168, 1038, 802, 628, 598, 2431, 338, 528, 1251, 835, 110, 2039, 746, 908, 204, 1219, 2148, 2055, 1065, 662, 1584, 1850, 2155, 153, 947, 549, 313, 1643, 1825, 2418, 2441, 1457, 307, 448, 1135, 751, 1261, 1252, 1299, 576, 1630, 373, 242, 430, 1554, 2443, 2212, 434, 687, 749, 320, 2508, 1841, 1312, 291, 2558, 1091, 1685, 678, 1089, 1328, 1331, 2314, 886, 1645, 55, 421, 463, 254, 906, 1929, 660, 199, 756, 2187, 359, 1032, 300, 2467, 1847, 655, 1950, 914, 2514, 129, 935, 2142, 843, 1996, 1727, 2506, 1210, 993, 1894, 548, 791, 1908, 1374, 450, 2261, 73, 546, 2181, 1752, 2226, 497, 1956, 216, 651, 1615, 621, 1039, 415, 1076, 633, 1549, 2621, 1874, 1670, 1992, 2466, 801, 810, 704, 1139, 293, 7, 664, 265, 813, 758, 1362, 2413, 2394, 1478, 266, 136, 2455, 403, 2219, 2048, 1966, 1764, 2423, 2535, 1388, 626, 2087, 2195, 2193, 477, 940, 555, 2346, 139, 971, 1231, 151, 1570, 2599, 121, 2589, 929, 1079, 2618, 236, 539, 873, 1861, 2391, 2620, 1361, 1110, 351, 1168, 531, 909, 2083, 1619, 690, 39, 349, 1492, 2547, 6, 798, 241, 1877, 2571, 178, 282, 290, 213, 794, 820, 1483, 125, 355, 1605, 2228, 2406, 401, 1293, 438, 1129, 515, 1472, 543, 2551, 1030, 2550, 1493, 2026, 1440, 1494, 923, 2041, 627, 1963, 2043, 363, 107, 1870, 1198, 1090, 816, 992, 2342, 2020, 1608, 983, 392, 1054, 1925, 1356, 1604, 1315, 2362, 1988, 1263, 2428, 1084, 1635, 1980, 1233, 765, 1779, 2437, 1017, 2101, 2452, 972, 2522, 1905, 1656, 1435, 2031, 611, 1143, 877, 1031, 725, 1311, 214, 1102, 2184, 1559, 1249, 1455, 1818, 688, 1532, 989, 56, 428, 1099, 894, 1497, 48, 2583, 1081, 2147, 10, 2084, 615, 1330, 1441, 2546, 1393, 1856, 2292, 707, 1769, 1195, 1485, 1935, 1878, 2018, 652, 1629, 949, 1467, 742, 1597, 1245, 1074, 2491, 1598, 960, 665, 2211, 1973, 1481, 1969, 1459, 1603, 1901, 2120, 235, 1990, 2002, 1916, 1700, 2572, 956, 1744, 2403, 1638, 1835, 328, 1735, 2042, 532, 868, 1014, 2196, 563, 1132, 706, 1482, 1207, 1490, 285, 1571, 1407, 134, 1648, 2517, 522, 201, 1044, 510, 2573, 1320, 2286, 53, 1295, 881, 1118, 2224, 763, 1552, 1708, 1834, 66, 140, 599, 1176, 1417, 1582, 1095, 1631, 1096, 2266, 2123, 1462, 2627, 1944, 2488, 2412, 2167, 1911, 217, 2213, 2022, 2560, 20, 1413, 1324, 1408, 2559, 478, 603, 15, 768, 2378, 1194, 413, 2205, 977, 1259, 1187, 1337, 1574, 986, 147, 2450, 946, 2009, 1620, 2139, 577, 2316, 2256, 1254, 64, 2062, 2047, 1923, 1959, 1360, 715, 1589, 1852, 2282, 566, 792, 1747, 710, 2233, 1951, 1426, 197, 1987, 1283, 568, 71, 1881, 2121, 1379, 487, 1373, 2384, 1590, 788, 17, 697, 1498, 1213, 834, 317, 249, 717, 2569, 875, 446, 860, 2387, 1863, 1225, 2283, 1004, 2056, 1896, 253, 27, 2464, 1907, 1737, 1229, 1119, 1366, 1821, 1609, 475, 455, 1199, 370, 609, 1335, 1500, 251, 1826, 2489, 1109, 1898, 1037, 2453, 208, 2285, 2465, 1985, 1566, 1375, 2463, 495, 437, 2421, 587, 2221, 1967, 1853, 1243, 263, 198, 3, 1383, 1961, 2459, 324, 2382, 2561, 999, 649, 159, 1345, 1232, 2612, 1522, 191, 1733, 2133, 724, 565, 1239, 1527, 1843, 1864, 1155, 1613, 299, 2151, 37, 2538, 1272, 1382, 149, 1587, 2439, 194, 2408, 1514, 799, 2502, 218, 2432, 844, 1885, 2371, 210, 1148, 2097, 542, 592, 1390, 395, 1775, 2543, 1288, 2085, 1196, 1438, 995, 99, 2076, 1469, 1808, 1150, 65, 1865, 1434, 2171, 2469, 2615, 1740, 1411, 1658, 496, 2231, 721, 1749, 1662, 1449, 2327, 486, 2278, 2037, 246, 62, 1145, 2, 1758, 2426, 1714, 1804, 1696, 353, 641, 396, 493, 329, 2549, 591, 1359, 384, 54, 775, 2625, 1085, 414, 1524, 1555, 636, 2149, 1946, 2623, 2040, 1739, 1573, 2500, 1829, 1290, 1309, 1070, 188, 2313, 2069, 2415, 2534, 691, 807, 2027, 46, 489, 847, 2520, 1468, 812, 2059, 1892, 2122, 193, 967, 504, 1887, 2144, 1970, 1387, 2567, 1974, 1997, 391, 825, 2017, 2045, 934, 857, 1180, 1854, 2357, 1831, 1012, 2242, 25, 509, 2061, 2315, 2476, 480, 31, 1840, 948, 481, 795, 1977, 1376, 2442, 1836, 8, 1226, 92, 804, 637, 2462, 932, 2301, 2325, 2143, 994, 1265, 196, 762, 776, 1027, 394, 716, 723, 879, 1142, 230, 2094, 1673, 1066, 618, 424, 51, 459, 1013, 676, 2075, 1652, 1550, 622, 352, 2080, 653, 2158, 1748, 336, 939, 1364, 1174, 1161, 2198, 163, 1131, 2019, 625, 385, 435, 1241, 43, 2060, 1052, 18, 2202, 192, 1972, 152, 2189, 112, 2474, 1157, 700, 1101, 2252, 928, 1833, 582, 357, 1842, 596, 2410, 2220, 1167, 2180, 1253, 2448, 2414, 1057, 240, 2000, 2162, 2269, 343, 2104, 2247, 1857, 1913, 1097, 2318, 2585, 225, 372, 2607, 1451, 1236, 1439, 381, 766, 69, 964, 567, 1915, 2077, 530, 2177, 1855, 852, 416, 334, 1323, 322, 672, 141, 1025, 2359, 1844, 1201, 750, 1414, 1443, 2536, 2596, 105, 1464, 1206, 60, 1722, 2338, 2273, 382, 422, 226, 2267, 183, 1108, 1872, 1982, 2244, 780, 2150, 2204, 2554, 1447, 1989, 1647, 1059, 1460, 2487, 1028, 1448, 1489, 1370, 921, 517, 2358, 1267, 1385, 1230, 1126, 2365, 2093, 1813, 1641, 467, 1711, 1531, 2336, 127, 1717, 2183, 536, 377, 705, 588, 2299, 1341, 2210, 2199, 296, 2427, 520, 161, 1560, 247, 2510, 2548, 1316, 374, 1310, 2088, 2132, 722, 506, 2207, 35, 466, 1240, 1773, 2058, 1954, 1336, 1399, 1237, 1392, 1999, 2478, 709, 1976, 1104, 1397, 2071, 2402, 1934, 1596, 1487, 1533, 90, 757, 2157, 1227, 1586, 2161, 2265, 1594, 1795, 222, 2374, 1463, 2516, 1294, 1679, 310, 1529, 1120, 1794, 822, 1409, 2272, 854, 1544, 2340, 2275, 1300, 1280, 891, 2063, 229, 490, 2364, 145, 366, 2329, 2001, 173, 2542, 1055, 1780, 142, 547, 1371, 1845, 1699, 1454, 2518, 2553, 2073, 503, 1056, 870, 52, 805, 1627, 237, 97, 174, 269, 257, 2270, 172, 243, 1415, 1307, 866, 867, 1791, 2477, 1697, 1505, 2249, 1561, 2385, 133, 579, 874, 143, 1223, 44, 32, 1553, 447, 137, 1621, 1798, 2174, 2070, 1536, 1389, 255, 2539, 2305, 2125, 23, 1940, 1178, 454, 2030, 260, 620, 888, 2417, 2348, 1329, 1209, 180, 2344, 1790, 2106, 1998, 2317, 1672, 2400, 1386, 2490, 1579, 2597, 2246, 741, 1203, 1624, 734, 1064, 2280, 744, 2262, 491, 74, 2366, 2613, 1474, 516, 409, 1757, 303, 674, 2310, 2227, 998, 745, 656, 1803, 1416, 2388, 893, 533, 1921, 1675, 1565, 1073, 2353, 573, 223, 2349, 1867, 2192, 903, 1275, 345, 1029, 1928, 855, 1504, 2449, 1732, 341, 2541, 1368, 885, 1920, 1979, 1050, 1033, 617, 1208, 2194, 1172, 2263, 571, 1326, 1107, 2424, 1820, 1660, 1726, 583, 1642, 2216, 364, 164, 1049, 1893, 2592, 1092, 1394, 157, 369, 277, 1184, 2399, 1276, 773, 1193, 1327, 590, 1602, 2446, 309, 2379, 410, 925, 836, 1601, 2311, 330, 1682, 258, 2396, 389, 950, 449, 1421, 1427, 2584, 941, 1068, 2320, 323, 1466, 1964, 2468, 2352, 1695, 2361, 1799, 2259, 826, 2407, 2011, 1924, 168, 1690, 393, 1086, 1848, 2209, 962, 61, 2154, 1784, 1600, 708, 2078, 1446, 111, 292, 1292, 2257, 58, 1112, 1958, 1016, 1650, 1866, 1269, 2145, 2284, 1810, 294, 2501, 713, 1296, 1250, 1668, 2118, 545, 1547, 1811, 1537, 2356, 271, 402, 2606, 1442, 1530, 441, 1495, 2425, 2616, 1540, 550, 1268, 521, 736, 314, 976, 239, 411, 969, 689, 1000, 701, 753, 1906, 1041, 738, 2297, 2138, 327, 2473, 88, 837, 1308, 1768, 985, 1002, 526, 514, 1785, 1644, 654, 1134, 1519, 445, 912, 49, 2498, 1353, 1346, 2287, 24, 1507, 553, 80, 1903, 1318, 2586, 728, 1367, 1192, 2250, 2375, 206, 1886, 186, 2034, 1471, 2436, 2105, 102, 2512, 2029, 2033, 128, 634, 245, 433, 2579, 275, 2028, 1083, 1410, 2590, 963, 537, 2007, 1904, 2092, 1521, 1756, 899, 483, 942, 2309, 2509, 1429, 1202, 1453, 311, 2235, 1380, 321, 1851, 2130, 427, 784, 1525, 2430, 1022, 312, 2215, 2341, 2322, 106, 955, 594, 2354, 815, 160, 1156, 1137, 1228, 67, 2404, 2303, 1766, 2507, 1750, 12, 897, 2127, 2533, 1305, 1777, 2460, 1612, 1625, 2035, 50, 2609, 219, 1042, 1759, 1122, 297, 1681, 1218, 1058, 2188, 658, 817, 1910, 714, 811, 1114, 2531, 42, 233, 931, 1664, 910, 898, 1745, 227, 1141, 2005, 2214, 2614, 640, 2483, 1333, 1781, 968, 2619, 1479, 661, 2555, 1995, 21, 1258, 1516, 2319, 1444, 507, 1588, 2447, 1577, 978, 2236, 578, 905, 1539, 2409, 179, 954, 278, 1502, 72, 378, 1704, 554, 1121, 417, 337, 1124, 1067, 256, 2243, 397, 1400, 1048, 2274, 1179, 1809, 1860, 693, 123, 224, 631, 786, 534, 519, 856, 1404, 1461, 279, 1680, 1654, 1846, 1007, 2218, 1880, 680, 761, 308, 344, 1372, 1106, 1282, 2355, 2010, 1884, 1140, 2102, 936, 535, 2225, 1542, 2255, 2484, 1883, 30, 1128, 953, 2381, 541, 692, 1162, 332, 1637, 1743, 556, 2461, 1797, 2025, 354, 838, 2054, 442, 1473, 360, 823, 1045, 89, 2444, 1713, 1941, 957, 1488, 572, 871, 2234, 232, 1812, 190, 1742, 154, 2300, 1633, 2038, 1616, 252, 919, 1663, 2575, 861, 1278, 657, 1922, 841, 1183, 2611, 575, 81, 98, 1523, 876, 1510, 1753, 2107, 318, 1152, 781, 839, 1902, 833, 1646, 671, 1738, 846, 1815, 2217, 560, 144, 1347, 2373, 1610, 40, 1968, 1949, 1465, 2254, 1691, 2166, 1557, 250, 638, 1094, 1838, 1890, 2479, 1024, 2116, 1728, 1279, 2578, 2100, 440, 158, 1264, 2290, 1639, 699, 1741, 431, 2626, 1496, 1023, 821, 1562, 551, 1585, 2237, 109, 720, 1778, 1993, 1671, 500, 1420, 850, 2457, 2333, 1026, 650, 607, 2295, 86, 831, 379, 1858, 580, 276, 1936, 1350, 2566, 1396, 1830, 1592, 1349, 1424, 451, 900, 1686, 2622, 171, 1729, 1509, 1428, 22, 770, 418, 1879, 78, 1082, 2564, 2525, 1403, 347, 47, 2260, 1363, 904, 2332, 1953, 1306, 759, 1792, 386, 1009, 613, 1859, 648, 982, 1543, 593, 1234, 970, 404, 2420, 917, 419, 702, 1702, 1651, 2350, 1978, 1452, 2594, 685, 9, 1113, 1277, 2470, 2245, 1832, 2511, 2529, 231, 405, 1719, 331, 600, 2172, 1667, 2395, 1800, 1618, 958, 2383, 524, 122, 406, 1378, 782, 2281, 2170, 2134, 2591, 205, 2505, 1100, 1020, 1423, 911, 2492, 2182, 614, 268, 471, 432, 1765, 2335, 2141, 211, 1077, 845, 666, 33, 1770, 2185, 262, 1689, 1287, 388, 1632, 1433, 1010, 138, 184, 1515, 0, 19, 1694, 2601, 1432, 1205, 735, 667, 14, 2131, 462, 1061, 1677, 996, 2617, 552, 1257, 1322, 1186, 642, 2230, 1991, 1707, 170, 1018, 1477, 132, 2206, 2393, 2389, 1701, 1221, 2445, 2624, 1762, 1684, 1255, 2582, 681, 589, 2588, 212, 371, 1270, 16, 966, 70, 2203, 1807, 1563, 2472, 1470, 316, 832, 34, 1158, 1583, 878, 2565, 729, 425, 632, 1170, 2494, 771, 1051, 439, 800, 731, 1703, 119, 1772, 1897, 484, 1164, 339, 228, 2321, 356, 669, 458, 115, 284, 387, 943, 732, 182, 1062, 2153, 2540, 1133, 1298, 1151, 1931, 1789, 997, 2377, 2197, 286, 980, 1247, 476, 2593, 975, 1406, 130, 1932, 456, 1182, 1868, 2164, 1001, 2066, 2021, 342, 261, 2049, 1291, 1882, 1036, 2351, 2108, 785, 189, 87, 712, 1189, 984, 1801, 1217, 2293, 1593, 315, 348, 747, 234, 1317, 207, 2065, 2298, 1709, 274, 733, 148, 973, 2099, 1019, 1659, 863, 79, 1712, 864, 1822, 2493, 2345, 1981, 1849, 244, 499, 2258, 883, 1828, 1721, 803, 1369, 176, 564, 1093, 2397, 2330, 639, 383, 1344, 1377, 2392, 880, 2367, 1260, 585, 1715, 38, 118, 1720, 872, 1545, 1518, 809, 902, 2595, 2307, 1281, 1343, 104, 2232, 2532, 2312, 11, 2339, 1617, 586, 557, 498, 1098, 1450, 1534, 2405, 2008, 827, 2323, 668, 2429, 1823, 2014, 529, 2003, 922, 945, 2023, 1984, 1626, 2095, 2603, 1425, 2163, 1006, 84, 696, 325, 1891, 2012, 1123, 1888, 1021, 1354, 1130, 1491, 1771, 2563, 2326, 1304, 2291, 1782, 1197, 679, 1548, 1005, 2523, 2580, 675, 2334, 1793, 485, 1526, 2537, 2451, 1786, 76, 1043, 828, 915, 779, 45, 423, 1567, 2046, 2241, 2173, 889, 1580, 1419, 1755, 2324, 1541, 162, 703, 1154, 511, 2238, 301, 399, 604, 1724, 1788, 2004, 1015, 298, 2090, 569, 1508, 1262, 2557, 787, 694, 408, 319, 1528, 2568, 1185, 502, 740, 1506, 1716, 1933, 913, 304, 358, 830, 2306, 806, 965, 1190, 2135, 961, 2605, 2485, 1806, 974, 1634, 1869, 1040, 117, 1339, 2552, 987, 1948, 335, 1938, 767, 2229, 1053, 783, 2111, 789, 558, 1165, 113, 1876, 2482, 1078, 333, 1734, 2136, 1204, 518, 1875, 645, 561, 468, 2057, 1952, 853, 907, 2610, 737, 1965, 156, 93, 1436, 2248, 2015, 990, 1649, 501, 2064, 2570, 2179, 2140, 1325, 760, 209, 1776, 114, 1824, 1839, 1395, 1955, 1937, 1581, 1248, 1912, 1816, 452, 1718, 1599, 1796, 1943, 682, 505, 131, 1736, 2044, 1235, 187, 1576, 1692, 1060, 629, 1220, 2082, 730, 1087, 135, 1238, 68, 1175, 488, 461, 849, 1614, 1499, 2576, 2368, 1358, 2128, 2370, 1080, 2016, 1918, 1927, 1412, 1575, 1274, 1873, 1666, 305, 1917, 1926, 647, 1297, 2458, 59, 595, 1919, 350, 2369, 1398, 1558, 2112, 944, 2544, 2289, 2086, 777, 2110, 288, 429, 398, 1047, 166, 1569, 203, 1127, 1827, 527, 2036, 26, 1381, 116, 774, 1242, 1754, 605, 1746, 754, 2386, 920, 472, 1607, 1930, 1501, 584, 1564, 2497, 2296, 85, 851, 2201, 2169, 1355, 460, 790, 1069, 1348, 2481, 1319, 103, 1688, 570, 1520, 220, 1244, 612, 1480, 2526, 412, 165, 2129, 346, 2178, 200, 1475, 77, 29, 663, 1365, 2067, 2175, 2581, 2363, 1693, 2271, 2343, 2191, 512, 365, 2608, 559, 2096, 2089, 1486, 2251, 2152, 2440, 13, 1116, 797, 36, 2398, 630, 508, 1623, 272, 2628, 1146, 1191, 1802, 1994, 544, 2119, 2454, 1805, 1115, 108, 2530, 259, 719, 991, 677, 2562, 2513, 1402, 1222, 2288, 215, 1661, 2160, 1188, 525, 1535, 1271, 2239, 1105, 426, 1636, 988, 2277, 2223, 101, 376, 2222, 927, 1153, 2308, 951, 2114, 829, 2051, 2495, 1302, 482, 2115, 1034, 390, 1939, 2328, 1163, 1256, 1215, 1246, 2360, 608, 1655, 933, 2052, 2124, 2521, 469, 727, 1160, 1, 581, 238, 57, 606, 1705, 2117, 264, 938, 1285, 155, 2013, 1513, 407, 1200, 1103, 146, 1556, 673, 1011, 273, 1430, 1767, 1266, 1458, 896, 643, 601, 124, 1181, 1819, 175, 2176, 1578, 2416, 1837, 2253, 1611, 1957, 1653, 884, 1063, 281, 616, 2053, 1774, 2380, 764, 540, 1214, 267, 361, 295, 1725, 1008, 1814, 1332, 698, 1169, 981, 2068, 368, 362, 1942, 1211, 1706, 1914, 1284, 1303, 2401, 2433, 1159, 2577, 150, 2347, 1783, 670, 952, 1314, 2302, 1787, 2556, 2527, 2074, 1511, 1171, 1657, 1900, 1431, 1401, 2079, 1899, 1321, 752, 1591, 1595, 2499, 1422, 2156, 1286, 2024, 2113, 2515, 1216, 306, 726, 126, 1962, 248, 858, 1147, 2545, 2503, 1071, 169, 1075, 1760, 892, 1173, 602, 718, 1551, 1710, 778, 2208, 882, 1683, 1761, 1338, 2091, 283, 2240, 2137, 1895, 1117, 2390, 2600, 1418, 1889, 2159, 2411, 280, 1289, 2604, 494, 1340, 959, 769, 380, 82, 739, 464, 1971, 1945, 711, 686, 2126, 1909, 465, 683, 695, 808, 1476, 2480, 796, 167, 1166, 287, 1342, 918, 457, 865, 755, 375, 41, 610, 895, 1072, 2081, 1751, 1698, 2376, 270, 2574, 1947, 340, 1212, 818, 420, 120, 1606, 1111, 1035, 2006, 1871, 916, 2103, 1986, 848, 1669, 185, 1512, 597, 1484, 2279, 195, 2186, 91, 624, 1674, 793, 1301, 1975, 1676, 2456, 1538, 1357, 1273, 443, 748, 2200, 367, 1763, 1351, 63, 289, 1622, 926, 814, 623, 95, 83, 96, 2294, 470, 2109, 1313, 1723, 1003, 1352, 890, 2276, 2331, 901, 1665, 2304, 1678, 659, 513, 1572, 75, 538, 444, 930, 1517, 479, 2435, 474, 302, 400, 1138, 2524, 644, 937, 1568, 562, 1730, 2504, 619, 2190, 840, 819, 1437, 2264, 1503, 2602, 1817, 221, 1983, 2496, 1628, 2438, 1405, 473, 1334, 2072, 924, 2419, 1125, 646, 887, 1177, 635, 2528, 2098, 181, 1149, 2422, 1546, 2372, 824, 202, 2471, 743, 1640, 2050, 1144, 574, 1445, 523, 1862, 94, 2434, 4, 684, 5, 436, 2475, 1136, 842],
    _phantom: core::marker::PhantomData,
};

pub(super) static LOWER_NAME_TO_IDX: PhfMap<[u8], u16> = PhfMap {
    key: 10656210946825422025,
    disps: &[(0, 231), (0, 12), (0, 241), (0, 14), (0, 7), (0, 208), (0, 177), (0, 3), (0, 33), (0, 1), (0, 20), (0, 51), (0, 12), (0, 28), (0, 33), (0, 84), (0, 31), (0, 139), (0, 0), (0, 156), (0, 2), (0, 377), (0, 35), (0, 39), (0, 0), (0, 146), (0, 8), (0, 0), (0, 150), (0, 115), (0, 56), (0, 15), (0, 11), (0, 89), (0, 5), (0, 0), (0, 34), (0, 12), (0, 33), (0, 13), (0, 7), (0, 335), (0, 536), (0, 214), (0, 395), (0, 136), (0, 0), (0, 90), (0, 57), (0, 3), (0, 13), (0, 317), (0, 732), (0, 7), (0, 11), (0, 121), (0, 155), (0, 214), (0, 1), (0, 222), (0, 209), (0, 13), (0, 386), (0, 38), (0, 78), (0, 130), (0, 2), (0, 4), (0, 15), (0, 31), (0, 28), (0, 0), (0, 437), (0, 250), (0, 71), (0, 64), (0, 99), (0, 0), (0, 7), (0, 4), (0, 28), (0, 14), (0, 1215), (0, 320), (0, 0), (0, 4), (0, 0), (0, 106), (0, 0), (0, 0), (0, 416), (0, 2), (0, 213), (0, 12), (0, 71), (0, 6), (0, 5), (0, 128), (0, 666), (0, 12), (0, 8), (0, 4), (0, 3), (0, 183), (0, 143), (0, 340), (0, 272), (0, 1), (0, 8), (0, 559), (0, 0), (0, 46), (1, 127), (0, 0), (0, 307), (0, 71), (0, 125), (0, 515), (0, 173), (0, 0), (0, 1), (0, 53), (0, 120), (0, 19), (0, 0), (0, 4), (0, 232), (0, 15), (1, 16), (0, 975), (0, 3), (0, 17), (0, 24), (0, 0), (0, 65), (0, 164), (0, 455), (0, 215), (0, 187), (0, 83), (0, 68), (0, 28), (0, 0), (0, 81), (0, 36), (0, 469), (0, 2), (0, 206), (0, 378), (0, 4), (0, 291), (0, 60), (0, 98), (0, 1313), (0, 202), (0, 0), (0, 3), (0, 3), (0, 551), (0, 705), (0, 4), (0, 53), (0, 23), (0, 6), (0, 432), (0, 2), (0, 53), (0, 14), (0, 23), (0, 0), (0, 17), (0, 128), (0, 1016), (0, 162), (0, 72), (0, 136), (0, 975), (0, 484), (0, 58), (0, 10), (0, 845), (0, 1111), (0, 671), (0, 538), (0, 145), (0, 0), (0, 1), (0, 21), (0, 0), (0, 179), (1, 1), (0, 500), (0, 14), (0, 374), (0, 670), (0, 48), (0, 12), (0, 31), (0, 136), (0, 0), (0, 26), (0, 0), (0, 11), (0, 1), (0, 9), (0, 99), (0, 0), (0, 0), (0, 1), (0, 5), (0, 3), (0, 87), (0, 194), (0, 39), (0, 48), (0, 3), (0, 146), (0, 22), (0, 3), (0, 0), (0, 208), (0, 81), (0, 4), (0, 0), (0, 719), (0, 21), (0, 761), (0, 3), (0, 19), (0, 462), (0, 1187), (0, 0), (0, 2), (0, 386), (0, 118), (0, 0), (0, 2), (0, 1), (0, 46), (0, 1), (0, 4), (0, 12), (0, 221), (1, 580), (0, 408), (0, 2), (0, 2), (0, 8), (0, 758), (0, 0), (0, 12), (0, 27), (0, 0), (0, 3), (0, 27), (0, 196), (0, 427), (0, 40), (0, 0), (0, 1021), (0, 142), (0, 106), (0, 15), (0, 0), (0, 742), (0, 61), (0, 993), (0, 487), (0, 12), (0, 2230), (1, 1286), (0, 4), (0, 102), (0, 8), (0, 111), (0, 45), (0, 68), (0, 8), (0, 1), (0, 33), (0, 25), (0, 1157), (0, 2), (0, 364), (0, 25), (0, 176), (0, 1529), (0, 22), (0, 13), (0, 131), (0, 72), (0, 1), (0, 134), (0, 5), (0, 150), (0, 584), (0, 21), (0, 2), (0, 86), (0, 1353), (0, 1095), (1, 217), (0, 16), (0, 0), (0, 0), (0, 19), (0, 1364), (0, 283), (0, 34), (0, 1666), (0, 12), (0, 28), (0, 107), (0, 593), (0, 19), (0, 151), (0, 448), (0, 411), (0, 1790), (0, 806), (0, 2), (0, 85), (0, 32), (0, 1519), (0, 8), (0, 1067), (0, 30), (0, 1378), (0, 0), (0, 255), (0, 20), (0, 179), (0, 244), (0, 9), (0, 131), (0, 238), (0, 352), (0, 22), (0, 42), (0, 786), (0, 1151), (0, 71), (0, 382), (0, 11), (0, 782), (0, 7), (0, 26), (0, 144), (0, 868), (0, 616), (0, 0), (0, 1397), (0, 91), (0, 0), (0, 27), (0, 22), (0, 0), (0, 25), (1, 1112), (0, 1270), (0, 2096), (0, 868), (0, 5), (0, 1811), (0, 2), (0, 79), (0, 7), (0, 18), (0, 25), (0, 53), (0, 249), (0, 1), (0, 253), (0, 1194), (0, 870), (0, 9), (0, 3), (0, 0), (0, 2), (0, 5), (0, 20), (0, 78), (0, 1125), (0, 1107), (0, 12), (0, 87), (0, 183), (0, 949), (0, 332), (1, 667), (0, 953), (0, 0), (0, 2), (0, 3), (0, 11), (0, 194), (1, 30), (0, 1107), (0, 1004), (0, 165), (0, 12), (0, 126), (0, 118), (0, 5), (0, 54), (0, 6), (0, 1), (0, 0), (0, 1), (0, 1235), (0, 1715), (0, 977), (0, 11), (0, 0), (0, 642), (0, 8), (0, 260), (0, 1012), (0, 2), (0, 1), (0, 354), (0, 6), (0, 0), (0, 10), (0, 1435), (0, 223), (0, 126), (0, 921), (0, 3), (2, 649), (0, 25), (0, 24), (0, 101), (0, 208), (0, 318), (0, 0), (0, 777), (0, 160), (1, 378), (0, 1151), (1, 1231), (0, 301), (0, 219), (0, 1), (0, 1), (2, 1810), (0, 148), (0, 1), (0, 1287), (0, 550), (0, 15), (0, 10), (0, 30), (1, 359), (0, 8), (0, 274), (0, 16), (0, 610), (0, 242), (0, 363), (0, 1328), (0, 1183), (0, 1055), (1, 634), (3, 1741), (0, 124), (0, 105), (0, 1277), (0, 77), (0, 1462), (0, 959), (0, 44), (0, 301), (0, 196), (0, 1420), (1, 1208), (0, 254), (0, 227), (0, 1454), (1, 792), (0, 1233), (0, 7), (0, 577), (0, 756), (0, 125), (0, 8), (0, 36), (0, 10), (0, 1734), (0, 0), (1, 280), (1, 1212), (0, 0), (3, 2081), (0, 369), (0, 219), (0, 483), (1, 601), (7, 153), (0, 602), (3, 1629), (5, 957), (0, 30), (1, 1070), (0, 477), (0, 33), (0, 94), (1, 1423), (0, 618), (0, 0), (5, 520), (0, 53)],
    map: &[2537, 2522, 2441, 1729, 778, 1559, 719, 2449, 13, 2589, 2600, 351, 2052, 76, 2416, 2261, 2594, 996, 606, 1999, 1021, 601, 921, 252, 1682, 1669, 1517, 1307, 202, 2474, 644, 245, 1737, 2609, 810, 1379, 886, 80, 425, 754, 1273, 2401, 1759, 1589, 172, 604, 801, 650, 2133, 2189, 65, 1805, 1576, 254, 249, 377, 669, 1859, 2315, 2272, 1513, 361, 401, 1996, 362, 434, 1983, 2524, 389, 176, 657, 2023, 869, 330, 2607, 1628, 1186, 1913, 1998, 2248, 1320, 1830, 1336, 1860, 115, 723, 788, 1029, 771, 358, 2265, 1165, 785, 1534, 316, 1783, 1144, 2006, 1530, 2172, 740, 761, 2549, 2463, 1931, 893, 1203, 2054, 2613, 2236, 910, 347, 2139, 769, 267, 2412, 2434, 122, 323, 1119, 591, 1941, 2420, 119, 2101, 925, 510, 1347, 1050, 2009, 487, 888, 315, 2038, 1035, 663, 1309, 1985, 2407, 1168, 941, 8, 636, 2220, 2561, 593, 1989, 976, 1384, 1809, 1924, 1590, 1959, 2106, 2535, 7, 1563, 1876, 816, 1883, 2273, 1761, 1612, 2523, 890, 560, 366, 999, 171, 485, 709, 2559, 1213, 876, 1080, 2439, 1245, 1840, 1966, 1158, 416, 121, 99, 1652, 2190, 2490, 1819, 1515, 1371, 1020, 2405, 1079, 1363, 1074, 907, 1974, 2070, 84, 739, 553, 2464, 2363, 2604, 1041, 981, 1889, 1410, 1190, 603, 625, 1012, 2329, 2425, 2598, 1427, 332, 1435, 605, 1311, 431, 1594, 2340, 2165, 1950, 1178, 694, 1096, 31, 1939, 1201, 490, 1599, 1125, 928, 2501, 1975, 1314, 2071, 741, 1429, 407, 1898, 1915, 124, 253, 1591, 979, 1337, 2171, 2285, 1893, 1057, 1523, 2221, 2027, 6, 394, 2011, 1990, 1937, 774, 1417, 683, 673, 2061, 1141, 2018, 1071, 1211, 2547, 1075, 956, 2117, 918, 62, 328, 1060, 2213, 2228, 863, 1155, 2253, 858, 1826, 2124, 1938, 222, 0, 2287, 664, 2212, 2437, 1166, 1874, 1501, 1231, 1256, 2307, 2102, 895, 1997, 1624, 98, 2057, 213, 2113, 2496, 1305, 2310, 439, 428, 2413, 711, 682, 1290, 619, 1385, 1154, 432, 1372, 1242, 2282, 1368, 753, 359, 722, 367, 962, 916, 2255, 1034, 509, 1070, 1067, 564, 2410, 414, 597, 2436, 2251, 1318, 2473, 915, 1921, 1901, 2195, 2599, 1283, 192, 2320, 1218, 826, 2214, 1546, 1611, 1691, 1567, 256, 805, 638, 555, 1196, 1498, 2215, 2194, 320, 2411, 2572, 263, 950, 2152, 451, 2323, 724, 1836, 2252, 504, 675, 1288, 1910, 2163, 1036, 1199, 1229, 744, 716, 1741, 749, 2222, 1979, 927, 2421, 795, 1854, 732, 1316, 1185, 177, 735, 1701, 2378, 482, 2154, 2243, 2318, 1536, 1613, 2096, 843, 2225, 1037, 640, 966, 2513, 408, 2417, 1949, 1044, 2312, 1969, 898, 802, 1115, 2493, 325, 106, 2618, 1206, 958, 168, 2349, 2079, 961, 844, 2423, 662, 2476, 2103, 1822, 978, 1677, 2104, 2584, 1422, 2077, 1557, 1932, 30, 804, 2359, 680, 934, 1294, 2532, 2497, 481, 2074, 2043, 2316, 1692, 896, 456, 2290, 681, 242, 1286, 1837, 2534, 1424, 371, 2461, 2415, 1993, 554, 1884, 627, 445, 184, 1833, 733, 1360, 1811, 130, 224, 2515, 1707, 1801, 807, 391, 34, 1431, 1046, 2514, 854, 762, 2563, 2375, 1499, 2239, 2311, 2540, 349, 815, 2227, 2471, 764, 157, 173, 1170, 2442, 868, 2308, 321, 1437, 2314, 800, 758, 1988, 1392, 1891, 1508, 16, 1658, 2126, 187, 1192, 327, 1813, 499, 645, 2489, 565, 2110, 1107, 29, 436, 503, 1238, 1739, 2596, 1087, 423, 1617, 2488, 2402, 1208, 355, 2536, 1306, 1660, 955, 94, 2430, 1495, 2456, 2053, 654, 2580, 2587, 1289, 2020, 2368, 1918, 1321, 840, 1364, 2546, 1747, 870, 2551, 1955, 1180, 1683, 1304, 879, 2400, 1926, 2092, 123, 2091, 1595, 2387, 2419, 2335, 1181, 1052, 1381, 1308, 2155, 781, 1367, 906, 501, 2249, 2066, 1662, 857, 2075, 2219, 2545, 930, 2327, 1030, 2624, 313, 2173, 2247, 2007, 2260, 894, 2558, 559, 103, 841, 1177, 2539, 1664, 938, 846, 770, 798, 2438, 15, 1230, 2617, 2408, 2206, 1092, 257, 991, 21, 817, 1881, 1390, 1132, 1968, 1005, 2134, 2244, 1981, 751, 95, 2268, 594, 2132, 2036, 793, 2208, 369, 970, 1870, 1056, 1421, 2324, 885, 2453, 2591, 1641, 368, 1532, 1423, 2620, 1655, 182, 561, 2021, 331, 929, 731, 2404, 1817, 902, 2455, 2325, 825, 1271, 1373, 631, 2516, 834, 2603, 178, 1418, 2332, 1671, 2495, 1625, 1868, 435, 993, 2058, 2136, 806, 2188, 1545, 2426, 1971, 2149, 1687, 105, 1156, 1663, 684, 2357, 2144, 1228, 329, 2342, 648, 1234, 1350, 2181, 1209, 1935, 1090, 1845, 365, 2443, 2034, 195, 2448, 2271, 1378, 90, 266, 2291, 718, 2479, 2164, 1520, 1940, 219, 9, 1709, 2193, 2286, 901, 390, 360, 2140, 1947, 1880, 2362, 611, 1313, 2005, 914, 2209, 2299, 1225, 1930, 1101, 243, 2481, 1043, 1659, 1856, 364, 311, 1962, 1877, 1094, 2326, 1195, 2346, 1857, 881, 2422, 1048, 2123, 1395, 908, 883, 1560, 1374, 1346, 791, 1540, 548, 2491, 2356, 736, 707, 317, 712, 1370, 667, 2004, 2427, 2276, 1869, 2367, 651, 1189, 1630, 400, 1301, 2505, 772, 174, 167, 1882, 410, 2090, 1549, 1503, 1820, 2562, 2414, 2628, 1404, 1251, 1832, 2147, 803, 2424, 1162, 668, 1807, 1200, 1013, 1145, 1927, 660, 643, 402, 1066, 2396, 1689, 1111, 265, 622, 2555, 1205, 849, 1679, 943, 1386, 2502, 1666, 1171, 2014, 1632, 2032, 1281, 2035, 699, 607, 2289, 129, 2211, 427, 293, 882, 794, 352, 1704, 244, 1415, 2321, 1825, 1902, 1152, 220, 12, 1449, 2183, 1040, 2046, 1609, 792, 2593, 1525, 1506, 2131, 346, 102, 1933, 114, 1715, 1693, 1278, 379, 2602, 1723, 1241, 1091, 871, 1317, 1182, 1016, 1175, 1380, 2352, 1366, 1014, 1252, 1193, 1583, 1064, 3, 1204, 1319, 188, 2302, 2146, 1279, 2177, 1657, 2135, 2381, 909, 2360, 2487, 2128, 2328, 63, 633, 1853, 2230, 1821, 646, 2203, 912, 980, 2098, 2001, 2577, 324, 1389, 721, 2353, 388, 830, 1839, 2065, 1674, 1239, 1255, 1945, 2569, 128, 1216, 2319, 1443, 1637, 713, 505, 939, 290, 1398, 251, 2406, 2157, 275, 1743, 1032, 757, 1623, 2129, 700, 1082, 2322, 867, 2627, 1957, 1244, 1550, 1653, 2232, 1702, 734, 831, 750, 179, 1269, 679, 557, 1645, 1622, 1202, 1387, 1690, 697, 704, 1529, 2263, 973, 853, 1916, 1953, 1982, 992, 641, 887, 1250, 437, 1103, 746, 931, 96, 656, 1753, 1538, 703, 563, 2234, 1253, 1779, 1509, 247, 306, 2466, 500, 995, 820, 693, 1146, 2174, 592, 1787, 933, 2180, 1561, 632, 2454, 1577, 951, 2398, 596, 655, 952, 1831, 1579, 1310, 1602, 1295, 1214, 2185, 276, 1587, 710, 2467, 1065, 1502, 1719, 1849, 1243, 972, 2192, 2444, 1751, 100, 708, 2044, 892, 2538, 11, 2305, 495, 2317, 2076, 2429, 281, 1920, 194, 1400, 1684, 1451, 2125, 196, 1892, 1963, 131, 1356, 2118, 1018, 766, 1285, 2338, 1397, 1528, 615, 2446, 1102, 2055, 1063, 1958, 963, 1588, 1377, 1605, 864, 2204, 937, 920, 1524, 2259, 2280, 1554, 2520, 2279, 1339, 1629, 1412, 1352, 1233, 1923, 1697, 429, 2611, 169, 4, 1246, 1047, 116, 1088, 2067, 1823, 1970, 945, 1217, 198, 1123, 1986, 2294, 1713, 2142, 2585, 1212, 199, 185, 326, 779, 765, 1951, 990, 1866, 1846, 2450, 822, 2073, 919, 1793, 2013, 2167, 2386, 1678, 2162, 949, 2588, 1531, 2296, 1767, 1116, 659, 1585, 856, 1153, 1287, 1522, 987, 1670, 1222, 1685, 608, 1757, 1354, 1089, 2506, 2384, 1797, 1011, 2331, 223, 1633, 730, 2059, 2566, 1003, 403, 1568, 2026, 1232, 241, 2100, 1, 489, 1634, 392, 598, 218, 2483, 1864, 2161, 1085, 1184, 2048, 438, 2257, 2119, 2107, 1977, 2469, 2298, 1221, 1686, 1942, 447, 562, 1965, 2240, 2022, 1455, 2361, 1994, 2262, 278, 2081, 2385, 1872, 2395, 175, 412, 2069, 1025, 2564, 357, 1419, 1562, 1344, 2615, 1838, 2313, 1871, 1086, 1055, 2374, 387, 1896, 1505, 1330, 1727, 1775, 2470, 2063, 10, 767, 913, 1496, 851, 1608, 1863, 2121, 595, 117, 484, 661, 14, 2533, 1858, 1000, 1643, 1172, 1076, 614, 1547, 295, 1254, 1852, 848, 1789, 702, 1544, 874, 1348, 1841, 1382, 1960, 2334, 599, 1197, 1964, 1326, 2130, 2264, 1873, 1121, 944, 903, 889, 880, 1992, 1904, 738, 2029, 998, 763, 486, 1174, 1646, 24, 2616, 1157, 2095, 492, 340, 1334, 483, 309, 111, 965, 1582, 1023, 386, 1642, 2510, 454, 1987, 314, 2484, 488, 2156, 796, 1327, 2373, 2078, 2482, 2097, 2595, 639, 344, 2508, 997, 2344, 790, 884, 1215, 776, 629, 2300, 2138, 2345, 215, 613, 2025, 264, 705, 2301, 1695, 1176, 2431, 1584, 1083, 214, 1610, 1850, 706, 1297, 449, 676, 1104, 808, 1980, 2394, 1618, 2281, 1763, 2623, 109, 1300, 875, 2527, 17, 312, 341, 2056, 2480, 82, 2565, 1323, 832, 2127, 2578, 1548, 911, 1922, 1897, 2143, 494, 760, 33, 2382, 2554, 2003, 2619, 2389, 1058, 430, 922, 2150, 905, 861, 726, 670, 421, 2343, 1578, 2015, 189, 821, 1696, 728, 1500, 1084, 1956, 2148, 1237, 2242, 837, 1335, 1626, 1497, 1093, 422, 216, 68, 1533, 440, 125, 971, 759, 289, 940, 2440, 1973, 1887, 1518, 1340, 2068, 2626, 2608, 1342, 932, 983, 2567, 1878, 652, 2526, 677, 1031, 2500, 2304, 2560, 678, 395, 1099, 1292, 1077, 891, 1861, 1580, 2541, 1260, 2614, 755, 2573, 780, 2292, 1656, 27, 1661, 2592, 1375, 782, 2223, 1592, 1597, 70, 1824, 2477, 2610, 2531, 1293, 479, 1280, 380, 2576, 2337, 685, 87, 1847, 924, 1262, 1445, 2233, 508, 2017, 1521, 497, 1312, 1675, 1138, 2622, 2170, 1665, 1143, 748, 1967, 1556, 453, 1638, 2582, 2579, 1911, 558, 642, 1282, 73, 2012, 1420, 2153, 1843, 212, 2550, 1331, 1136, 1543, 1668, 72, 1106, 602, 768, 609, 1284, 838, 1163, 948, 1745, 1619, 89, 1100, 104, 2468, 2108, 2051, 634, 600, 691, 1394, 811, 418, 2457, 1008, 799, 1681, 2229, 845, 404, 2275, 409, 1391, 318, 2116, 2250, 701, 1028, 1274, 2485, 2269, 1027, 1651, 1952, 2529, 1001, 1264, 2082, 1555, 2105, 1108, 1210, 74, 1010, 2256, 1257, 1357, 904, 1038, 2145, 1600, 745, 1537, 2266, 630, 727, 1324, 1073, 1514, 1078, 777, 1908, 823, 2574, 385, 1991, 671, 350, 342, 2084, 1944, 612, 1946, 448, 1315, 2041, 1586, 2601, 2120, 1978, 2277, 2087, 977, 396, 2198, 101, 1620, 1526, 1068, 383, 743, 818, 1925, 279, 2499, 83, 32, 1223, 1865, 1700, 953, 1519, 1785, 2028, 1062, 378, 2543, 183, 637, 2176, 2369, 2355, 399, 1388, 334, 2339, 1573, 2199, 714, 1565, 442, 1906, 1349, 1504, 92, 687, 899, 1247, 507, 1128, 1929, 2024, 2581, 2445, 959, 2283, 2186, 1948, 1551, 132, 2166, 1358, 23, 729, 812, 406, 618, 947, 954, 877, 64, 1733, 181, 690, 2297, 2, 1393, 85, 842, 2169, 2465, 897, 1650, 2088, 363, 274, 480, 1649, 688, 935, 1566, 2447, 61, 2333, 2040, 1984, 1019, 1258, 2366, 775, 2254, 974, 1049, 628, 1773, 2293, 2050, 635, 2517, 197, 1159, 382, 443, 1731, 1569, 2452, 2238, 1408, 75, 126, 1332, 25, 1053, 2008, 1834, 1353, 2179, 1164, 2295, 1627, 2519, 1240, 2039, 923, 2226, 2031, 1112, 1259, 292, 491, 2548, 2350, 698, 2200, 1126, 814, 2278, 982, 417, 833, 2210, 866, 1291, 850, 248, 22, 1803, 686, 917, 2267, 653, 1615, 1795, 747, 2099, 2575, 1512, 1453, 2159, 1934, 986, 1325, 1220, 2390, 333, 786, 1081, 1735, 2379, 1022, 960, 1113, 2380, 1886, 2016, 1059, 988, 1542, 860, 444, 1581, 2586, 1362, 556, 1149, 1553, 2428, 2002, 1345, 221, 926, 2521, 1552, 2597, 1277, 2217, 1699, 1604, 1219, 2049, 66, 77, 1236, 2336, 1851, 1263, 1867, 2590, 2270, 1842, 2151, 1098, 1598, 2168, 1161, 1361, 1717, 1042, 1570, 942, 319, 200, 624, 1024, 2241, 1120, 493, 370, 1265, 828, 1188, 862, 1140, 2451, 1961, 1571, 2556, 1054, 2399, 1648, 97, 985, 616, 1376, 2030, 1033, 2246, 1227, 26, 1261, 1296, 2207, 1117, 2354, 1328, 2525, 2072, 873, 426, 322, 2288, 1667, 969, 610, 2062, 86, 1777, 1769, 1909, 2388, 1365, 2196, 872, 2086, 1006, 190, 1835, 2109, 506, 1680, 1698, 1338, 1888, 2175, 1907, 2528, 1267, 19, 689, 186, 1122, 2552, 433, 2570, 1694, 1160, 2503, 413, 936, 1433, 2557, 852, 110, 112, 1899, 1954, 855, 2330, 2365, 446, 2089, 1928, 1302, 170, 696, 720, 67, 79, 2284, 621, 2530, 2583, 2060, 1771, 1829, 2459, 1015, 620, 2347, 81, 384, 1640, 2383, 2571, 502, 2433, 1972, 1593, 2397, 1383, 381, 1322, 787, 1369, 717, 1636, 348, 356, 2237, 1026, 1130, 28, 1341, 1173, 2303, 2085, 69, 2377, 1844, 1194, 2542, 496, 784, 308, 1894, 1270, 2235, 1616, 180, 1574, 1142, 658, 1276, 674, 1711, 2160, 2376, 498, 108, 88, 1676, 946, 405, 2348, 1359, 1606, 2568, 1855, 2231, 984, 1516, 2371, 2000, 2472, 742, 343, 2202, 1333, 2178, 120, 783, 1875, 1061, 968, 288, 1890, 2158, 1755, 1298, 2509, 2494, 71, 1895, 975, 1124, 1224, 441, 2224, 2093, 419, 2392, 2512, 1118, 1045, 1539, 5, 393, 2621, 2432, 1725, 1631, 2182, 2258, 957, 2019, 1329, 1441, 255, 1639, 2351, 2141, 797, 666, 2201, 773, 424, 1510, 1134, 250, 1266, 345, 2115, 1654, 1601, 1635, 2274, 2372, 376, 2010, 1558, 1039, 2083, 2504, 18, 2544, 626, 2197, 1191, 2612, 2064, 1183, 1268, 2370, 1943, 450, 1919, 824, 127, 672, 2460, 1095, 1167, 649, 2358, 1912, 1721, 411, 2478, 372, 964, 865, 2245, 859, 1198, 1976, 1169, 835, 752, 1607, 1179, 1248, 2403, 415, 2625, 2553, 2218, 789, 1914, 20, 1069, 1235, 1621, 989, 2306, 994, 91, 836, 1917, 1002, 307, 2511, 1351, 455, 2364, 1596, 1207, 1494, 2606, 452, 1004, 1703, 715, 107, 2462, 2184, 1425, 2205, 737, 1249, 1905, 1644, 1705, 1575, 1511, 398, 113, 692, 847, 193, 665, 277, 217, 725, 2122, 1848, 1799, 2309, 1827, 2458, 2409, 623, 2341, 647, 617, 1226, 1355, 280, 2047, 2475, 756, 2518, 1105, 1541, 2042, 420, 1828, 260, 1275, 2492, 839, 1879, 1815, 1303, 1672, 1791, 2391, 1614, 2507, 1603, 1995, 246, 93, 878, 310, 1765, 2045, 1299, 1110, 2435, 2191, 829, 2187, 1017, 118, 1885, 78, 2094, 1447, 1072, 1272, 1097, 1564, 2486, 1343, 2216, 2111, 1862, 1749, 1137, 819, 1527, 1507, 291, 1936, 1187, 1402, 1688, 191, 2418, 2605, 827, 1903, 1109, 695, 1781, 1572, 2080, 1439, 2037, 2112, 1900, 2033, 1114, 1051, 2114, 1647, 1535, 2137, 397, 1673, 813, 2393, 809, 900, 1396, 967],
    _phantom: core::marker::PhantomData,
};

pub(super) static KEYSYM_TO_UPPER_KEYSYM: PhfMap<u32, KeysymCaseMapping> = PhfMap {
    key: 10656210946825422025,
    disps: &[(0, 0), (0, 39), (0, 0), (0, 0), (0, 3), (0, 3), (0, 7), (0, 0), (0, 1), (0, 24), (0, 10), (0, 5), (0, 0), (0, 5), (0, 6), (0, 68), (0, 0), (0, 19), (0, 1), (0, 15), (0, 2), (0, 34), (0, 38), (0, 3), (0, 11), (0, 95), (0, 3), (0, 1), (0, 9), (0, 4), (0, 6), (0, 103), (0, 1), (0, 1), (0, 6), (0, 0), (0, 77), (0, 0), (0, 89), (0, 23), (0, 2), (0, 0), (0, 0), (0, 99), (0, 0), (0, 7), (0, 18), (0, 145), (0, 2), (0, 64), (0, 13), (0, 6), (0, 31), (0, 82), (0, 0), (0, 0), (0, 37), (0, 16), (0, 0), (0, 151), (0, 54), (0, 6), (3, 50), (0, 6)],
    map: &[KeysymCaseMapping { keysym: 0x06d8, other: 0x000006f8 }, KeysymCaseMapping { keysym: 0x03b3, other: 0x000003a3 }, KeysymCaseMapping { keysym: 0x06ae, other: 0x000006be }, KeysymCaseMapping { keysym: 0x01ea, other: 0x000001ca }, KeysymCaseMapping { keysym: 0x07e1, other: 0x000007c1 }, KeysymCaseMapping { keysym: 0x06c5, other: 0x000006e5 }, KeysymCaseMapping { keysym: 0x07e5, other: 0x000007c5 }, KeysymCaseMapping { keysym: 0x02fd, other: 0x000002dd }, KeysymCaseMapping { keysym: 0x06cb, other: 0x000006eb }, KeysymCaseMapping { keysym: 0x06d0, other: 0x000006f0 }, KeysymCaseMapping { keysym: 0x06c4, other: 0x000006e4 }, KeysymCaseMapping { keysym: 0x03b5, other: 0x000003a5 }, KeysymCaseMapping { keysym: 0x01b5, other: 0x000001a5 }, KeysymCaseMapping { keysym: 0x00fc, other: 0x000000dc }, KeysymCaseMapping { keysym: 0x06a3, other: 0x000006b3 }, KeysymCaseMapping { keysym: 0x06d7, other: 0x000006f7 }, KeysymCaseMapping { keysym: 0x00f3, other: 0x000000d3 }, KeysymCaseMapping { keysym: 0x01fe, other: 0x000001de }, KeysymCaseMapping { keysym: 0x06ac, other: 0x000006bc }, KeysymCaseMapping { keysym: 0x07e2, other: 0x000007c2 }, KeysymCaseMapping { keysym: 0x06c6, other: 0x000006e6 }, KeysymCaseMapping { keysym: 0x06a5, other: 0x000006b5 }, KeysymCaseMapping { keysym: 0x01bb, other: 0x000001ab }, KeysymCaseMapping { keysym: 0x01e8, other: 0x000001c8 }, KeysymCaseMapping { keysym: 0x01f1, other: 0x000001d1 }, KeysymCaseMapping { keysym: 0x07f3, other: 0x000007d2 }, KeysymCaseMapping { keysym: 0x00ef, other: 0x000000cf }, KeysymCaseMapping { keysym: 0x06a1, other: 0x000006b1 }, KeysymCaseMapping { keysym: 0x06d5, other: 0x000006f5 }, KeysymCaseMapping { keysym: 0x00f1, other: 0x000000d1 }, KeysymCaseMapping { keysym: 0x06c9, other: 0x000006e9 }, KeysymCaseMapping { keysym: 0x06aa, other: 0x000006ba }, KeysymCaseMapping { keysym: 0x03bc, other: 0x000003ac }, KeysymCaseMapping { keysym: 0x00fa, other: 0x000000da }, KeysymCaseMapping { keysym: 0x03e7, other: 0x000003c7 }, KeysymCaseMapping { keysym: 0x00e0, other: 0x000000c0 }, KeysymCaseMapping { keysym: 0x00e1, other: 0x000000c1 }, KeysymCaseMapping { keysym: 0x07ef, other: 0x000007cf }, KeysymCaseMapping { keysym: 0x00ed, other: 0x000000cd }, KeysymCaseMapping { keysym: 0x07f1, other: 0x000007d1 }, KeysymCaseMapping { keysym: 0x07e4, other: 0x000007c4 }, KeysymCaseMapping { keysym: 0x03f9, other: 0x000003d9 }, KeysymCaseMapping { keysym: 0x06a4, other: 0x000006b4 }, KeysymCaseMapping { keysym: 0x07b9, other: 0x000007a9 }, KeysymCaseMapping { keysym: 0x02bc, other: 0x000002ac }, KeysymCaseMapping { keysym: 0x01b6, other: 0x000001a6 }, KeysymCaseMapping { keysym: 0x02b1, other: 0x000002a1 }, KeysymCaseMapping { keysym: 0x06d3, other: 0x000006f3 }, KeysymCaseMapping { keysym: 0x01b1, other: 0x000001a1 }, KeysymCaseMapping { keysym: 0x02e5, other: 0x000002c5 }, KeysymCaseMapping { keysym: 0x01bc, other: 0x000001ac }, KeysymCaseMapping { keysym: 0x02f8, other: 0x000002d8 }, KeysymCaseMapping { keysym: 0x00f8, other: 0x000000d8 }, KeysymCaseMapping { keysym: 0x00ee, other: 0x000000ce }, KeysymCaseMapping { keysym: 0x01f8, other: 0x000001d8 }, KeysymCaseMapping { keysym: 0x07b4, other: 0x000007a4 }, KeysymCaseMapping { keysym: 0x06a8, other: 0x000006b8 }, KeysymCaseMapping { keysym: 0x00eb, other: 0x000000cb }, KeysymCaseMapping { keysym: 0x07e9, other: 0x000007c9 }, KeysymCaseMapping { keysym: 0x00ff, other: 0x000013be }, KeysymCaseMapping { keysym: 0x01fb, other: 0x000001db }, KeysymCaseMapping { keysym: 0x07eb, other: 0x000007cb }, KeysymCaseMapping { keysym: 0x03bf, other: 0x000003bd }, KeysymCaseMapping { keysym: 0x06ad, other: 0x000006bd }, KeysymCaseMapping { keysym: 0x06c2, other: 0x000006e2 }, KeysymCaseMapping { keysym: 0x00fd, other: 0x000000dd }, KeysymCaseMapping { keysym: 0x07ee, other: 0x000007ce }, KeysymCaseMapping { keysym: 0x01e6, other: 0x000001c6 }, KeysymCaseMapping { keysym: 0x01b3, other: 0x000001a3 }, KeysymCaseMapping { keysym: 0x07b2, other: 0x000007a2 }, KeysymCaseMapping { keysym: 0x00e6, other: 0x000000c6 }, KeysymCaseMapping { keysym: 0x06c0, other: 0x000006e0 }, KeysymCaseMapping { keysym: 0x02e6, other: 0x000002c6 }, KeysymCaseMapping { keysym: 0x00ec, other: 0x000000cc }, KeysymCaseMapping { keysym: 0x06d1, other: 0x000006f1 }, KeysymCaseMapping { keysym: 0x00f9, other: 0x000000d9 }, KeysymCaseMapping { keysym: 0x03ec, other: 0x000003cc }, KeysymCaseMapping { keysym: 0x01f5, other: 0x000001d5 }, KeysymCaseMapping { keysym: 0x00df, other: 0x01001e9e }, KeysymCaseMapping { keysym: 0x07f7, other: 0x000007d7 }, KeysymCaseMapping { keysym: 0x02f5, other: 0x000002d5 }, KeysymCaseMapping { keysym: 0x03fe, other: 0x000003de }, KeysymCaseMapping { keysym: 0x07b7, other: 0x000007a7 }, KeysymCaseMapping { keysym: 0x06ab, other: 0x000006bb }, KeysymCaseMapping { keysym: 0x06d2, other: 0x000006f2 }, KeysymCaseMapping { keysym: 0x00fb, other: 0x000000db }, KeysymCaseMapping { keysym: 0x03ba, other: 0x000003aa }, KeysymCaseMapping { keysym: 0x07b8, other: 0x000007a8 }, KeysymCaseMapping { keysym: 0x06df, other: 0x000006ff }, KeysymCaseMapping { keysym: 0x06ce, other: 0x000006ee }, KeysymCaseMapping { keysym: 0x07e8, other: 0x000007c8 }, KeysymCaseMapping { keysym: 0x06a6, other: 0x000006b6 }, KeysymCaseMapping { keysym: 0x00e4, other: 0x000000c4 }, KeysymCaseMapping { keysym: 0x00f6, other: 0x000000d6 }, KeysymCaseMapping { keysym: 0x07ed, other: 0x000007cd }, KeysymCaseMapping { keysym: 0x00ea, other: 0x000000ca }, KeysymCaseMapping { keysym: 0x07ec, other: 0x000007cc }, KeysymCaseMapping { keysym: 0x07bb, other: 0x000007ab }, KeysymCaseMapping { keysym: 0x00b5, other: 0x000007cc }, KeysymCaseMapping { keysym: 0x01f9, other: 0x000001d9 }, KeysymCaseMapping { keysym: 0x02bb, other: 0x000002ab }, KeysymCaseMapping { keysym: 0x01be, other: 0x000001ae }, KeysymCaseMapping { keysym: 0x07f9, other: 0x000007d9 }, KeysymCaseMapping { keysym: 0x03ef, other: 0x000003cf }, KeysymCaseMapping { keysym: 0x06dd, other: 0x000006fd }, KeysymCaseMapping { keysym: 0x07f6, other: 0x000007d6 }, KeysymCaseMapping { keysym: 0x00e2, other: 0x000000c2 }, KeysymCaseMapping { keysym: 0x03f2, other: 0x000003d2 }, KeysymCaseMapping { keysym: 0x06af, other: 0x000006bf }, KeysymCaseMapping { keysym: 0x03b6, other: 0x000003a6 }, KeysymCaseMapping { keysym: 0x07ea, other: 0x000007ca }, KeysymCaseMapping { keysym: 0x06c3, other: 0x000006e3 }, KeysymCaseMapping { keysym: 0x00f4, other: 0x000000d4 }, KeysymCaseMapping { keysym: 0x03f1, other: 0x000003d1 }, KeysymCaseMapping { keysym: 0x02b6, other: 0x000002a6 }, KeysymCaseMapping { keysym: 0x07b5, other: 0x000007a5 }, KeysymCaseMapping { keysym: 0x06a9, other: 0x000006b9 }, KeysymCaseMapping { keysym: 0x01ef, other: 0x000001cf }, KeysymCaseMapping { keysym: 0x06db, other: 0x000006fb }, KeysymCaseMapping { keysym: 0x07f4, other: 0x000007d4 }, KeysymCaseMapping { keysym: 0x07e7, other: 0x000007c7 }, KeysymCaseMapping { keysym: 0x06de, other: 0x000006fe }, KeysymCaseMapping { keysym: 0x03e0, other: 0x000003c0 }, KeysymCaseMapping { keysym: 0x01b9, other: 0x000001a9 }, KeysymCaseMapping { keysym: 0x07b3, other: 0x000007a3 }, KeysymCaseMapping { keysym: 0x06c1, other: 0x000006e1 }, KeysymCaseMapping { keysym: 0x01e0, other: 0x000001c0 }, KeysymCaseMapping { keysym: 0x01f2, other: 0x000001d2 }, KeysymCaseMapping { keysym: 0x00e9, other: 0x000000c9 }, KeysymCaseMapping { keysym: 0x00e7, other: 0x000000c7 }, KeysymCaseMapping { keysym: 0x00e8, other: 0x000000c8 }, KeysymCaseMapping { keysym: 0x06ca, other: 0x000006ea }, KeysymCaseMapping { keysym: 0x03bb, other: 0x000003ab }, KeysymCaseMapping { keysym: 0x02b9, other: 0x00000049 }, KeysymCaseMapping { keysym: 0x06dc, other: 0x000006fc }, KeysymCaseMapping { keysym: 0x07f2, other: 0x000007d2 }, KeysymCaseMapping { keysym: 0x00e5, other: 0x000000c5 }, KeysymCaseMapping { keysym: 0x01ba, other: 0x000001aa }, KeysymCaseMapping { keysym: 0x06d9, other: 0x000006f9 }, KeysymCaseMapping { keysym: 0x00f2, other: 0x000000d2 }, KeysymCaseMapping { keysym: 0x06cf, other: 0x000006ef }, KeysymCaseMapping { keysym: 0x06cc, other: 0x000006ec }, KeysymCaseMapping { keysym: 0x07f8, other: 0x000007d8 }, KeysymCaseMapping { keysym: 0x06c8, other: 0x000006e8 }, KeysymCaseMapping { keysym: 0x13bd, other: 0x000013bc }, KeysymCaseMapping { keysym: 0x03fd, other: 0x000003dd }, KeysymCaseMapping { keysym: 0x01f0, other: 0x000001d0 }, KeysymCaseMapping { keysym: 0x06a7, other: 0x000006b7 }, KeysymCaseMapping { keysym: 0x06d4, other: 0x000006f4 }, KeysymCaseMapping { keysym: 0x07f0, other: 0x000007d0 }, KeysymCaseMapping { keysym: 0x01bf, other: 0x000001af }, KeysymCaseMapping { keysym: 0x06a2, other: 0x000006b2 }, KeysymCaseMapping { keysym: 0x01e5, other: 0x000001c5 }, KeysymCaseMapping { keysym: 0x00f0, other: 0x000000d0 }, KeysymCaseMapping { keysym: 0x00e3, other: 0x000000c3 }, KeysymCaseMapping { keysym: 0x01ec, other: 0x000001cc }, KeysymCaseMapping { keysym: 0x07e6, other: 0x000007c6 }, KeysymCaseMapping { keysym: 0x06c7, other: 0x000006e7 }, KeysymCaseMapping { keysym: 0x06cd, other: 0x000006ed }, KeysymCaseMapping { keysym: 0x07b1, other: 0x000007a1 }, KeysymCaseMapping { keysym: 0x00f5, other: 0x000000d5 }, KeysymCaseMapping { keysym: 0x02fe, other: 0x000002de }, KeysymCaseMapping { keysym: 0x07e3, other: 0x000007c3 }, KeysymCaseMapping { keysym: 0x01e3, other: 0x000001c3 }, KeysymCaseMapping { keysym: 0x06da, other: 0x000006fa }, KeysymCaseMapping { keysym: 0x07f5, other: 0x000007d5 }, KeysymCaseMapping { keysym: 0x06d6, other: 0x000006f6 }, KeysymCaseMapping { keysym: 0x03f3, other: 0x000003d3 }, KeysymCaseMapping { keysym: 0x08f6, other: 0x01000191 }, KeysymCaseMapping { keysym: 0x00fe, other: 0x000000de }],
    _phantom: core::marker::PhantomData,
};

pub(super) static KEYSYM_TO_LOWER_KEYSYM: PhfMap<u32, KeysymCaseMapping> = PhfMap {
    key: 10656210946825422025,
    disps: &[(0, 43), (0, 55), (0, 3), (0, 3), (0, 16), (0, 3), (0, 1), (0, 2), (0, 2), (0, 5), (0, 6), (0, 5), (0, 2), (0, 19), (0, 0), (0, 2), (0, 7), (0, 0), (0, 0), (0, 25), (0, 10), (0, 50), (0, 18), (0, 2), (0, 1), (0, 51), (0, 20), (0, 9), (0, 9), (0, 1), (0, 28), (0, 4), (0, 103), (0, 0), (0, 87), (0, 0), (0, 0), (0, 6), (0, 125), (0, 74), (0, 0), (0, 8), (0, 80), (0, 39), (0, 103), (0, 11), (0, 8), (0, 28), (0, 140), (0, 3), (0, 39), (0, 0), (0, 3), (0, 1), (0, 0), (0, 0), (0, 16), (0, 3), (0, 1), (0, 1), (0, 21), (0, 35), (1, 84), (0, 60)],
    map: &[KeysymCaseMapping { keysym: 0x03cf, other: 0x000003ef }, KeysymCaseMapping { keysym: 0x06b4, other: 0x000006a4 }, KeysymCaseMapping { keysym: 0x06bb, other: 0x000006ab }, KeysymCaseMapping { keysym: 0x06b7, other: 0x000006a7 }, KeysymCaseMapping { keysym: 0x06f2, other: 0x000006d2 }, KeysymCaseMapping { keysym: 0x00d6, other: 0x000000f6 }, KeysymCaseMapping { keysym: 0x00dd, other: 0x000000fd }, KeysymCaseMapping { keysym: 0x01d8, other: 0x000001f8 }, KeysymCaseMapping { keysym: 0x00c1, other: 0x000000e1 }, KeysymCaseMapping { keysym: 0x01cf, other: 0x000001ef }, KeysymCaseMapping { keysym: 0x00c0, other: 0x000000e0 }, KeysymCaseMapping { keysym: 0x07cb, other: 0x000007eb }, KeysymCaseMapping { keysym: 0x03c0, other: 0x000003e0 }, KeysymCaseMapping { keysym: 0x03aa, other: 0x000003ba }, KeysymCaseMapping { keysym: 0x06f9, other: 0x000006d9 }, KeysymCaseMapping { keysym: 0x00c6, other: 0x000000e6 }, KeysymCaseMapping { keysym: 0x03de, other: 0x000003fe }, KeysymCaseMapping { keysym: 0x06be, other: 0x000006ae }, KeysymCaseMapping { keysym: 0x07d5, other: 0x000007f5 }, KeysymCaseMapping { keysym: 0x00d0, other: 0x000000f0 }, KeysymCaseMapping { keysym: 0x00c7, other: 0x000000e7 }, KeysymCaseMapping { keysym: 0x07d8, other: 0x000007f8 }, KeysymCaseMapping { keysym: 0x00ca, other: 0x000000ea }, KeysymCaseMapping { keysym: 0x06fc, other: 0x000006dc }, KeysymCaseMapping { keysym: 0x06f8, other: 0x000006d8 }, KeysymCaseMapping { keysym: 0x07cf, other: 0x000007ef }, KeysymCaseMapping { keysym: 0x01aa, other: 0x000001ba }, KeysymCaseMapping { keysym: 0x01d5, other: 0x000001f5 }, KeysymCaseMapping { keysym: 0x03d9, other: 0x000003f9 }, KeysymCaseMapping { keysym: 0x06f3, other: 0x000006d3 }, KeysymCaseMapping { keysym: 0x07a7, other: 0x000007b7 }, KeysymCaseMapping { keysym: 0x06b8, other: 0x000006a8 }, KeysymCaseMapping { keysym: 0x03cc, other: 0x000003ec }, KeysymCaseMapping { keysym: 0x06ff, other: 0x000006df }, KeysymCaseMapping { keysym: 0x01d9, other: 0x000001f9 }, KeysymCaseMapping { keysym: 0x03d2, other: 0x000003f2 }, KeysymCaseMapping { keysym: 0x07c2, other: 0x000007e2 }, KeysymCaseMapping { keysym: 0x06ed, other: 0x000006cd }, KeysymCaseMapping { keysym: 0x00da, other: 0x000000fa }, KeysymCaseMapping { keysym: 0x06fb, other: 0x000006db }, KeysymCaseMapping { keysym: 0x03ab, other: 0x000003bb }, KeysymCaseMapping { keysym: 0x06e3, other: 0x000006c3 }, KeysymCaseMapping { keysym: 0x06bf, other: 0x000006af }, KeysymCaseMapping { keysym: 0x06e0, other: 0x000006c0 }, KeysymCaseMapping { keysym: 0x01cc, other: 0x000001ec }, KeysymCaseMapping { keysym: 0x06fd, other: 0x000006dd }, KeysymCaseMapping { keysym: 0x01d2, other: 0x000001f2 }, KeysymCaseMapping { keysym: 0x07d9, other: 0x000007f9 }, KeysymCaseMapping { keysym: 0x06f6, other: 0x000006d6 }, KeysymCaseMapping { keysym: 0x00c4, other: 0x000000e4 }, KeysymCaseMapping { keysym: 0x07cc, other: 0x000007ec }, KeysymCaseMapping { keysym: 0x07c8, other: 0x000007e8 }, KeysymCaseMapping { keysym: 0x00d1, other: 0x000000f1 }, KeysymCaseMapping { keysym: 0x07a1, other: 0x000007b1 }, KeysymCaseMapping { keysym: 0x01c3, other: 0x000001e3 }, KeysymCaseMapping { keysym: 0x02ac, other: 0x000002bc }, KeysymCaseMapping { keysym: 0x06ba, other: 0x000006aa }, KeysymCaseMapping { keysym: 0x01a1, other: 0x000001b1 }, KeysymCaseMapping { keysym: 0x01ab, other: 0x000001bb }, KeysymCaseMapping { keysym: 0x00cb, other: 0x000000eb }, KeysymCaseMapping { keysym: 0x03d3, other: 0x000003f3 }, KeysymCaseMapping { keysym: 0x07d2, other: 0x000007f2 }, KeysymCaseMapping { keysym: 0x06b9, other: 0x000006a9 }, KeysymCaseMapping { keysym: 0x06ea, other: 0x000006ca }, KeysymCaseMapping { keysym: 0x00ce, other: 0x000000ee }, KeysymCaseMapping { keysym: 0x07a4, other: 0x000007b4 }, KeysymCaseMapping { keysym: 0x00d4, other: 0x000000f4 }, KeysymCaseMapping { keysym: 0x07ab, other: 0x000007bb }, KeysymCaseMapping { keysym: 0x06e6, other: 0x000006c6 }, KeysymCaseMapping { keysym: 0x02c5, other: 0x000002e5 }, KeysymCaseMapping { keysym: 0x00db, other: 0x000000fb }, KeysymCaseMapping { keysym: 0x02a6, other: 0x000002b6 }, KeysymCaseMapping { keysym: 0x03a5, other: 0x000003b5 }, KeysymCaseMapping { keysym: 0x01c6, other: 0x000001e6 }, KeysymCaseMapping { keysym: 0x02de, other: 0x000002fe }, KeysymCaseMapping { keysym: 0x03dd, other: 0x000003fd }, KeysymCaseMapping { keysym: 0x07cd, other: 0x000007ed }, KeysymCaseMapping { keysym: 0x06fa, other: 0x000006da }, KeysymCaseMapping { keysym: 0x06bc, other: 0x000006ac }, KeysymCaseMapping { keysym: 0x00de, other: 0x000000fe }, KeysymCaseMapping { keysym: 0x07c3, other: 0x000007e3 }, KeysymCaseMapping { keysym: 0x06f0, other: 0x000006d0 }, KeysymCaseMapping { keysym: 0x06b2, other: 0x000006a2 }, KeysymCaseMapping { keysym: 0x01c0, other: 0x000001e0 }, KeysymCaseMapping { keysym: 0x00c8, other: 0x000000e8 }, KeysymCaseMapping { keysym: 0x01a5, other: 0x000001b5 }, KeysymCaseMapping { keysym: 0x02d5, other: 0x000002f5 }, KeysymCaseMapping { keysym: 0x13bc, other: 0x000013bd }, KeysymCaseMapping { keysym: 0x06b3, other: 0x000006a3 }, KeysymCaseMapping { keysym: 0x00c5, other: 0x000000e5 }, KeysymCaseMapping { keysym: 0x07a5, other: 0x000007b5 }, KeysymCaseMapping { keysym: 0x07d6, other: 0x000007f6 }, KeysymCaseMapping { keysym: 0x07a8, other: 0x000007b8 }, KeysymCaseMapping { keysym: 0x06eb, other: 0x000006cb }, KeysymCaseMapping { keysym: 0x00d5, other: 0x000000f5 }, KeysymCaseMapping { keysym: 0x06ef, other: 0x000006cf }, KeysymCaseMapping { keysym: 0x00d8, other: 0x000000f8 }, KeysymCaseMapping { keysym: 0x06e1, other: 0x000006c1 }, KeysymCaseMapping { keysym: 0x01af, other: 0x000001bf }, KeysymCaseMapping { keysym: 0x06e4, other: 0x000006c4 }, KeysymCaseMapping { keysym: 0x01d0, other: 0x000001f0 }, KeysymCaseMapping { keysym: 0x03bd, other: 0x000003bf }, KeysymCaseMapping { keysym: 0x06b5, other: 0x000006a5 }, KeysymCaseMapping { keysym: 0x06ee, other: 0x000006ce }, KeysymCaseMapping { keysym: 0x03c7, other: 0x000003e7 }, KeysymCaseMapping { keysym: 0x07d0, other: 0x000007f0 }, KeysymCaseMapping { keysym: 0x02d8, other: 0x000002f8 }, KeysymCaseMapping { keysym: 0x07c7, other: 0x000007e7 }, KeysymCaseMapping { keysym: 0x06bd, other: 0x000006ad }, KeysymCaseMapping { keysym: 0x06f4, other: 0x000006d4 }, KeysymCaseMapping { keysym: 0x00c2, other: 0x000000e2 }, KeysymCaseMapping { keysym: 0x01ca, other: 0x000001ea }, KeysymCaseMapping { keysym: 0x01a9, other: 0x000001b9 }, KeysymCaseMapping { keysym: 0x06f7, other: 0x000006d7 }, KeysymCaseMapping { keysym: 0x07c9, other: 0x000007e9 }, KeysymCaseMapping { keysym: 0x06b6, other: 0x000006a6 }, KeysymCaseMapping { keysym: 0x06f1, other: 0x000006d1 }, KeysymCaseMapping { keysym: 0x06e8, other: 0x000006c8 }, KeysymCaseMapping { keysym: 0x07c6, other: 0x000007e6 }, KeysymCaseMapping { keysym: 0x03ac, other: 0x000003bc }, KeysymCaseMapping { keysym: 0x03d1, other: 0x000003f1 }, KeysymCaseMapping { keysym: 0x02a1, other: 0x000002b1 }, KeysymCaseMapping { keysym: 0x06e7, other: 0x000006c7 }, KeysymCaseMapping { keysym: 0x07c1, other: 0x000007e1 }, KeysymCaseMapping { keysym: 0x07a2, other: 0x000007b2 }, KeysymCaseMapping { keysym: 0x00d2, other: 0x000000f2 }, KeysymCaseMapping { keysym: 0x00c9, other: 0x000000e9 }, KeysymCaseMapping { keysym: 0x07d7, other: 0x000007f7 }, KeysymCaseMapping { keysym: 0x01d1, other: 0x000001f1 }, KeysymCaseMapping { keysym: 0x02a9, other: 0x00000069 }, KeysymCaseMapping { keysym: 0x06e5, other: 0x000006c5 }, KeysymCaseMapping { keysym: 0x00d9, other: 0x000000f9 }, KeysymCaseMapping { keysym: 0x02ab, other: 0x000002bb }, KeysymCaseMapping { keysym: 0x07a9, other: 0x000007b9 }, KeysymCaseMapping { keysym: 0x06e2, other: 0x000006c2 }, KeysymCaseMapping { keysym: 0x00cc, other: 0x000000ec }, KeysymCaseMapping { keysym: 0x01ac, other: 0x000001bc }, KeysymCaseMapping { keysym: 0x06fe, other: 0x000006de }, KeysymCaseMapping { keysym: 0x07d1, other: 0x000007f1 }, KeysymCaseMapping { keysym: 0x00dc, other: 0x000000fc }, KeysymCaseMapping { keysym: 0x07c5, other: 0x000007e5 }, KeysymCaseMapping { keysym: 0x00c3, other: 0x000000e3 }, KeysymCaseMapping { keysym: 0x01db, other: 0x000001fb }, KeysymCaseMapping { keysym: 0x06f5, other: 0x000006d5 }, KeysymCaseMapping { keysym: 0x01a3, other: 0x000001b3 }, KeysymCaseMapping { keysym: 0x00cf, other: 0x000000ef }, KeysymCaseMapping { keysym: 0x07ca, other: 0x000007ea }, KeysymCaseMapping { keysym: 0x02c6, other: 0x000002e6 }, KeysymCaseMapping { keysym: 0x07c4, other: 0x000007e4 }, KeysymCaseMapping { keysym: 0x06b1, other: 0x000006a1 }, KeysymCaseMapping { keysym: 0x06e9, other: 0x000006c9 }, KeysymCaseMapping { keysym: 0x07d4, other: 0x000007f4 }, KeysymCaseMapping { keysym: 0x01ae, other: 0x000001be }, KeysymCaseMapping { keysym: 0x03a3, other: 0x000003b3 }, KeysymCaseMapping { keysym: 0x07ce, other: 0x000007ee }, KeysymCaseMapping { keysym: 0x00d3, other: 0x000000f3 }, KeysymCaseMapping { keysym: 0x07a3, other: 0x000007b3 }, KeysymCaseMapping { keysym: 0x00cd, other: 0x000000ed }, KeysymCaseMapping { keysym: 0x01a6, other: 0x000001b6 }, KeysymCaseMapping { keysym: 0x01de, other: 0x000001fe }, KeysymCaseMapping { keysym: 0x13be, other: 0x000000ff }, KeysymCaseMapping { keysym: 0x03a6, other: 0x000003b6 }, KeysymCaseMapping { keysym: 0x02dd, other: 0x000002fd }, KeysymCaseMapping { keysym: 0x01c8, other: 0x000001e8 }, KeysymCaseMapping { keysym: 0x06ec, other: 0x000006cc }, KeysymCaseMapping { keysym: 0x01c5, other: 0x000001e5 }],
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

pub(super) static CHAR_TO_BESPOKE_IDX: PhfMap<char, u16> = PhfMap {
    key: 10656210946825422025,
    disps: &[(0, 334), (0, 73), (0, 4), (0, 42), (0, 6), (0, 0), (0, 7), (0, 4), (0, 0), (0, 1), (0, 0), (0, 2), (0, 1), (0, 2), (0, 101), (0, 19), (0, 6), (0, 104), (0, 2), (0, 0), (0, 16), (0, 15), (0, 2), (0, 79), (0, 9), (0, 3), (0, 22), (0, 64), (0, 9), (0, 1), (0, 69), (0, 6), (0, 0), (0, 116), (0, 4), (0, 11), (0, 30), (0, 65), (0, 0), (0, 101), (0, 4), (0, 42), (0, 41), (0, 5), (0, 2), (0, 94), (0, 168), (0, 21), (0, 57), (0, 31), (0, 13), (0, 8), (0, 1), (0, 191), (0, 69), (0, 188), (0, 153), (0, 10), (0, 10), (0, 17), (0, 17), (0, 5), (0, 164), (0, 5), (0, 12), (0, 74), (0, 13), (0, 0), (0, 14), (0, 0), (0, 0), (0, 283), (0, 12), (0, 32), (0, 73), (0, 0), (0, 0), (0, 1), (0, 4), (0, 0), (0, 0), (0, 3), (0, 0), (0, 99), (0, 1), (0, 0), (0, 61), (0, 33), (0, 39), (0, 31), (0, 10), (0, 0), (0, 0), (0, 14), (0, 1), (0, 0), (0, 1), (0, 7), (0, 24), (0, 33), (0, 87), (0, 14), (0, 23), (0, 1), (0, 1), (0, 2), (0, 144), (0, 95), (0, 4), (0, 10), (0, 224), (0, 108), (0, 6), (0, 101), (0, 1), (0, 19), (0, 2), (0, 8), (0, 151), (0, 0), (0, 88), (0, 69), (0, 192), (0, 54), (0, 190), (0, 77), (0, 0), (0, 8), (0, 284), (0, 33), (0, 86), (0, 771), (0, 150), (0, 117), (0, 187), (0, 233), (0, 656), (0, 54), (0, 13), (0, 434), (0, 0), (0, 103), (0, 29), (0, 22), (0, 362), (0, 59), (0, 4), (0, 141), (0, 23), (0, 0), (0, 595), (0, 339), (0, 173), (0, 50), (0, 1), (0, 1), (0, 3), (0, 770), (0, 38), (0, 396), (0, 66), (0, 1), (0, 1), (0, 10), (0, 0), (0, 294), (0, 157), (0, 0), (0, 10), (0, 16), (0, 652), (0, 212), (0, 115), (0, 2), (0, 728), (0, 0), (0, 1), (0, 10), (0, 4), (0, 44), (0, 0), (0, 0), (0, 105), (0, 2), (0, 2), (0, 4), (0, 203), (0, 19), (0, 0), (0, 47), (0, 1), (0, 137), (0, 32), (0, 303), (0, 146), (0, 98), (0, 0), (1, 164), (1, 0), (0, 43), (0, 3), (0, 45), (0, 0), (0, 31), (0, 69), (0, 39), (0, 181), (0, 2), (0, 690), (0, 184), (0, 7), (0, 29), (0, 17), (0, 26), (0, 3), (0, 21), (0, 718), (1, 54), (1, 648), (0, 38), (5, 537), (0, 320), (0, 615), (0, 0), (3, 147), (0, 613), (0, 248), (2, 163), (0, 22), (0, 10), (0, 347), (0, 38), (0, 71), (0, 15), (0, 0), (0, 143), (0, 39), (0, 14), (0, 57), (0, 670), (0, 1), (1, 651), (0, 282), (0, 18), (2, 432), (0, 89), (0, 2), (0, 88), (0, 3), (0, 0), (0, 118), (0, 386), (2, 258), (1, 763), (4, 766), (6, 661)],
    map: &[656, 107, 383, 574, 270, 175, 188, 682, 820, 554, 957, 307, 338, 411, 880, 727, 987, 399, 54, 945, 846, 517, 1000, 1, 851, 979, 606, 630, 600, 565, 76, 871, 343, 741, 124, 49, 892, 426, 721, 974, 19, 787, 571, 749, 665, 408, 627, 118, 374, 720, 347, 250, 937, 657, 492, 485, 458, 10, 907, 610, 755, 859, 661, 752, 391, 427, 73, 197, 538, 242, 108, 1010, 731, 165, 252, 489, 34, 8, 450, 510, 273, 139, 303, 432, 444, 251, 735, 309, 247, 227, 221, 1195, 241, 439, 51, 919, 588, 718, 525, 739, 711, 147, 27, 85, 938, 90, 493, 535, 753, 238, 65, 580, 451, 876, 392, 473, 915, 835, 785, 253, 594, 341, 101, 172, 15, 302, 799, 278, 923, 233, 185, 962, 530, 484, 888, 81, 501, 317, 558, 585, 503, 951, 59, 231, 24, 201, 889, 533, 319, 176, 217, 520, 351, 692, 640, 405, 545, 282, 38, 906, 189, 115, 220, 717, 946, 28, 905, 772, 136, 291, 702, 72, 494, 335, 1397, 852, 700, 703, 645, 516, 203, 109, 400, 623, 765, 843, 967, 976, 194, 671, 435, 453, 368, 980, 1006, 498, 649, 263, 568, 801, 20, 289, 740, 959, 380, 62, 754, 286, 582, 643, 603, 149, 984, 680, 614, 413, 22, 816, 202, 56, 344, 567, 96, 3, 283, 576, 793, 651, 459, 689, 881, 381, 663, 893, 882, 230, 410, 660, 860, 157, 144, 487, 832, 515, 143, 894, 169, 476, 513, 981, 887, 607, 762, 111, 724, 873, 546, 324, 812, 758, 293, 87, 870, 781, 868, 353, 589, 355, 64, 334, 348, 931, 631, 514, 792, 591, 462, 102, 82, 12, 310, 909, 924, 401, 123, 948, 130, 505, 91, 620, 508, 357, 21, 865, 628, 939, 491, 511, 342, 199, 339, 294, 274, 652, 840, 45, 521, 822, 989, 902, 653, 647, 215, 288, 861, 212, 534, 708, 921, 78, 611, 37, 746, 183, 133, 900, 285, 442, 53, 1188, 766, 956, 190, 495, 164, 696, 416, 968, 224, 837, 750, 29, 764, 394, 997, 596, 802, 17, 769, 174, 145, 667, 615, 187, 532, 1002, 691, 208, 370, 560, 751, 482, 292, 916, 216, 953, 277, 229, 681, 557, 883, 434, 475, 257, 809, 479, 407, 332, 784, 573, 854, 690, 932, 103, 191, 969, 30, 35, 308, 467, 760, 255, 705, 138, 356, 685, 982, 325, 402, 234, 254, 605, 540, 454, 112, 367, 232, 845, 75, 117, 196, 431, 549, 104, 608, 878, 536, 624, 207, 206, 867, 58, 84, 683, 986, 168, 480, 961, 419, 152, 316, 728, 457, 61, 895, 284, 259, 912, 862, 421, 940, 158, 92, 236, 803, 358, 579, 578, 146, 650, 933, 509, 488, 122, 570, 918, 295, 697, 267, 839, 345, 430, 5, 757, 768, 260, 593, 599, 834, 437, 996, 625, 814, 616, 320, 80, 817, 113, 896, 668, 518, 826, 89, 50, 23, 218, 925, 161, 744, 547, 386, 522, 581, 44, 838, 125, 779, 237, 584, 465, 36, 14, 428, 911, 736, 701, 162, 875, 941, 93, 529, 276, 120, 359, 613, 223, 245, 583, 553, 209, 395, 182, 975, 304, 601, 992, 179, 885, 523, 244, 963, 204, 559, 301, 693, 670, 786, 632, 55, 68, 677, 994, 710, 904, 366, 655, 240, 375, 958, 1005, 481, 184, 1190, 842, 167, 95, 695, 97, 382, 738, 412, 326, 855, 497, 279, 715, 995, 927, 805, 819, 2, 926, 884, 423, 729, 192, 336, 105, 684, 328, 519, 69, 983, 171, 153, 132, 63, 31, 477, 456, 694, 178, 950, 763, 180, 658, 712, 213, 524, 142, 362, 872, 575, 999, 404, 934, 722, 971, 32, 265, 119, 821, 572, 506, 271, 747, 321, 388, 811, 502, 300, 512, 662, 897, 114, 646, 890, 564, 543, 550, 140, 6, 198, 305, 372, 687, 707, 732, 528, 828, 11, 771, 504, 211, 970, 869, 1001, 337, 77, 644, 551, 641, 39, 743, 418, 248, 287, 908, 346, 609, 1007, 733, 942, 296, 371, 94, 947, 719, 988, 815, 129, 314, 935, 539, 490, 669, 377, 848, 886, 7, 396, 269, 349, 389, 66, 311, 836, 563, 595, 299, 898, 648, 964, 52, 173, 86, 794, 46, 313, 877, 258, 318, 955, 704, 360, 991, 847, 555, 25, 417, 163, 952, 40, 1187, 393, 384, 856, 249, 177, 205, 148, 350, 361, 406, 913, 16, 642, 928, 672, 186, 312, 849, 617, 998, 409, 70, 943, 424, 531, 246, 448, 460, 378, 699, 844, 767, 977, 329, 397, 474, 106, 385, 761, 363, 127, 561, 415, 298, 725, 116, 98, 369, 33, 972, 181, 569, 43, 57, 759, 154, 1191, 315, 471, 866, 364, 214, 306, 742, 659, 137, 499, 507, 770, 373, 552, 857, 151, 745, 195, 71, 829, 542, 4, 83, 679, 698, 1009, 420, 541, 598, 810, 929, 853, 219, 586, 891, 155, 48, 141, 879, 688, 414, 433, 425, 1008, 379, 222, 526, 47, 340, 446, 42, 60, 874, 813, 272, 936, 436, 88, 824, 686, 973, 590, 917, 901, 243, 469, 920, 390, 676, 748, 327, 235, 791, 18, 864, 99, 79, 13, 795, 621, 226, 297, 592, 612, 833, 483, 965, 200, 807, 121, 604, 556, 899, 275, 949, 160, 1004, 664, 960, 629, 486, 587, 734, 990, 228, 156, 403, 170, 463, 910, 678, 440, 756, 944, 674, 1003, 709, 239, 131, 74, 135, 262, 134, 850, 675, 398, 903, 783, 978, 331, 268, 496, 429, 966, 841, 26, 914, 566, 985, 597, 562, 730, 544, 537, 993, 290, 100, 673, 654, 41, 797, 256, 863, 387, 1189, 706, 376, 225, 330, 266, 210, 261, 954, 280, 323, 264, 500, 527, 858, 452, 626, 713, 922, 930, 830],
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

