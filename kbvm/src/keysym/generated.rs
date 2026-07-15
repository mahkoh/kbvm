use super::*;

#[cfg(test)]
pub(super) const LEN: usize = 2505;

pub(super) const LONGEST_NAME: usize = 30;

pub(super) static NAMES: &str = "NoSymbolspaceexclamquotedblnumbersigndollarpercentampersandapostrophequoterightparenleftparenrightasteriskpluscommaminusperiodslash0123456789colonsemicolonlessequalgreaterquestionatABCDEFGHIJKLMNOPQRSTUVWXYZbracketleftbackslashbracketrightasciicircumunderscoregravequoteleftabcdefghijklmnopqrstuvwxyzbraceleftbarbracerightasciitildenobreakspaceexclamdowncentsterlingcurrencyyenbrokenbarsectiondiaeresiscopyrightordfeminineguillemotleftguillemetleftnotsignhyphenregisteredmacrondegreeplusminustwosuperiorthreesuperioracutemuparagraphperiodcenteredcedillaonesuperiormasculineordmasculineguillemotrightguillemetrightonequarteronehalfthreequartersquestiondownAgraveAacuteAcircumflexAtildeAdiaeresisAringAECcedillaEgraveEacuteEcircumflexEdiaeresisIgraveIacuteIcircumflexIdiaeresisETHEthNtildeOgraveOacuteOcircumflexOtildeOdiaeresismultiplyOslashOobliqueUgraveUacuteUcircumflexUdiaeresisYacuteTHORNThornssharpagraveaacuteacircumflexatildeadiaeresisaringaeccedillaegraveeacuteecircumflexediaeresisigraveiacuteicircumflexidiaeresisethntildeograveoacuteocircumflexotildeodiaeresisdivisionoslashoobliqueugraveuacuteucircumflexudiaeresisyacutethornydiaeresisAogonekbreveLstrokeLcaronSacuteScaronScedillaTcaronZacuteZcaronZabovedotaogonekogoneklstrokelcaronsacutecaronscaronscedillatcaronzacutedoubleacutezcaronzabovedotRacuteAbreveLacuteCacuteCcaronEogonekEcaronDcaronDstrokeNacuteNcaronOdoubleacuteRcaronUringUdoubleacuteTcedillaracuteabrevelacutecacuteccaroneogonekecarondcarondstrokenacutencaronodoubleacutercaronuringudoubleacutetcedillaabovedotHstrokeHcircumflexIabovedotGbreveJcircumflexhstrokehcircumflexidotlessgbrevejcircumflexCabovedotCcircumflexGabovedotGcircumflexUbreveScircumflexcabovedotccircumflexgabovedotgcircumflexubrevescircumflexkrakappaRcedillaItildeLcedillaEmacronGcedillaTslashrcedillaitildelcedillaemacrongcedillatslashENGengAmacronIogonekEabovedotImacronNcedillaOmacronKcedillaUogonekUtildeUmacronamacroniogonekeabovedotimacronncedillaomacronkcedillauogonekutildeumacronoverlinekana_fullstopkana_openingbracketkana_closingbracketkana_commakana_conjunctivekana_middledotkana_WOkana_akana_ikana_ukana_ekana_okana_yakana_yukana_yokana_tsukana_tuprolongedsoundkana_Akana_Ikana_Ukana_Ekana_Okana_KAkana_KIkana_KUkana_KEkana_KOkana_SAkana_SHIkana_SUkana_SEkana_SOkana_TAkana_CHIkana_TIkana_TSUkana_TUkana_TEkana_TOkana_NAkana_NIkana_NUkana_NEkana_NOkana_HAkana_HIkana_FUkana_HUkana_HEkana_HOkana_MAkana_MIkana_MUkana_MEkana_MOkana_YAkana_YUkana_YOkana_RAkana_RIkana_RUkana_REkana_ROkana_WAkana_NvoicedsoundsemivoicedsoundArabic_commaArabic_semicolonArabic_question_markArabic_hamzaArabic_maddaonalefArabic_hamzaonalefArabic_hamzaonwawArabic_hamzaunderalefArabic_hamzaonyehArabic_alefArabic_behArabic_tehmarbutaArabic_tehArabic_thehArabic_jeemArabic_hahArabic_khahArabic_dalArabic_thalArabic_raArabic_zainArabic_seenArabic_sheenArabic_sadArabic_dadArabic_tahArabic_zahArabic_ainArabic_ghainArabic_tatweelArabic_fehArabic_qafArabic_kafArabic_lamArabic_meemArabic_noonArabic_haArabic_hehArabic_wawArabic_alefmaksuraArabic_yehArabic_fathatanArabic_dammatanArabic_kasratanArabic_fathaArabic_dammaArabic_kasraArabic_shaddaArabic_sukunSerbian_djeMacedonia_gjeCyrillic_ioUkrainian_ieUkranian_jeMacedonia_dseUkrainian_iUkranian_iUkrainian_yiUkranian_yiCyrillic_jeSerbian_jeCyrillic_ljeSerbian_ljeCyrillic_njeSerbian_njeSerbian_tsheMacedonia_kjeUkrainian_ghe_with_upturnByelorussian_shortuCyrillic_dzheSerbian_dzenumerosignSerbian_DJEMacedonia_GJECyrillic_IOUkrainian_IEUkranian_JEMacedonia_DSEUkrainian_IUkranian_IUkrainian_YIUkranian_YICyrillic_JESerbian_JECyrillic_LJESerbian_LJECyrillic_NJESerbian_NJESerbian_TSHEMacedonia_KJEUkrainian_GHE_WITH_UPTURNByelorussian_SHORTUCyrillic_DZHESerbian_DZECyrillic_yuCyrillic_aCyrillic_beCyrillic_tseCyrillic_deCyrillic_ieCyrillic_efCyrillic_gheCyrillic_haCyrillic_iCyrillic_shortiCyrillic_kaCyrillic_elCyrillic_emCyrillic_enCyrillic_oCyrillic_peCyrillic_yaCyrillic_erCyrillic_esCyrillic_teCyrillic_uCyrillic_zheCyrillic_veCyrillic_softsignCyrillic_yeruCyrillic_zeCyrillic_shaCyrillic_eCyrillic_shchaCyrillic_cheCyrillic_hardsignCyrillic_YUCyrillic_ACyrillic_BECyrillic_TSECyrillic_DECyrillic_IECyrillic_EFCyrillic_GHECyrillic_HACyrillic_ICyrillic_SHORTICyrillic_KACyrillic_ELCyrillic_EMCyrillic_ENCyrillic_OCyrillic_PECyrillic_YACyrillic_ERCyrillic_ESCyrillic_TECyrillic_UCyrillic_ZHECyrillic_VECyrillic_SOFTSIGNCyrillic_YERUCyrillic_ZECyrillic_SHACyrillic_ECyrillic_SHCHACyrillic_CHECyrillic_HARDSIGNGreek_ALPHAaccentGreek_EPSILONaccentGreek_ETAaccentGreek_IOTAaccentGreek_IOTAdieresisGreek_IOTAdiaeresisGreek_OMICRONaccentGreek_UPSILONaccentGreek_UPSILONdieresisGreek_OMEGAaccentGreek_accentdieresisGreek_horizbarGreek_alphaaccentGreek_epsilonaccentGreek_etaaccentGreek_iotaaccentGreek_iotadieresisGreek_iotaaccentdieresisGreek_omicronaccentGreek_upsilonaccentGreek_upsilondieresisGreek_upsilonaccentdieresisGreek_omegaaccentGreek_ALPHAGreek_BETAGreek_GAMMAGreek_DELTAGreek_EPSILONGreek_ZETAGreek_ETAGreek_THETAGreek_IOTAGreek_KAPPAGreek_LAMDAGreek_LAMBDAGreek_MUGreek_NUGreek_XIGreek_OMICRONGreek_PIGreek_RHOGreek_SIGMAGreek_TAUGreek_UPSILONGreek_PHIGreek_CHIGreek_PSIGreek_OMEGAGreek_alphaGreek_betaGreek_gammaGreek_deltaGreek_epsilonGreek_zetaGreek_etaGreek_thetaGreek_iotaGreek_kappaGreek_lamdaGreek_lambdaGreek_muGreek_nuGreek_xiGreek_omicronGreek_piGreek_rhoGreek_sigmaGreek_finalsmallsigmaGreek_tauGreek_upsilonGreek_phiGreek_chiGreek_psiGreek_omegaleftradicaltopleftradicalhorizconnectortopintegralbotintegralvertconnectortopleftsqbracketbotleftsqbrackettoprightsqbracketbotrightsqbrackettopleftparensbotleftparenstoprightparensbotrightparensleftmiddlecurlybracerightmiddlecurlybracetopleftsummationbotleftsummationtopvertsummationconnectorbotvertsummationconnectortoprightsummationbotrightsummationrightmiddlesummationlessthanequalnotequalgreaterthanequalintegralthereforevariationinfinitynablaapproximatesimilarequalifonlyifimpliesidenticalradicalincludedinincludesintersectionunionlogicalandlogicalorpartialderivativefunctionleftarrowuparrowrightarrowdownarrowblanksoliddiamondcheckerboardhtffcrlfnlvtlowrightcorneruprightcornerupleftcornerlowleftcornercrossinglineshorizlinescan1horizlinescan3horizlinescan5horizlinescan7horizlinescan9lefttrighttbotttoptvertbaremspaceenspaceem3spaceem4spacedigitspacepunctspacethinspacehairspaceemdashendashsignifblankellipsisdoubbaselinedotonethirdtwothirdsonefifthtwofifthsthreefifthsfourfifthsonesixthfivesixthscareoffigdashleftanglebracketdecimalpointrightanglebracketmarkeroneeighththreeeighthsfiveeighthsseveneighthstrademarksignaturemarktrademarkincircleleftopentrianglerightopentriangleemopencircleemopenrectangleleftsinglequotemarkrightsinglequotemarkleftdoublequotemarkrightdoublequotemarkprescriptionpermilleminutessecondslatincrosshexagramfilledrectbulletfilledlefttribulletfilledrighttribulletemfilledcircleemfilledrectenopencircbulletenopensquarebulletopenrectbulletopentribulletupopentribulletdownopenstarenfilledcircbulletenfilledsqbulletfilledtribulletupfilledtribulletdownleftpointerrightpointerclubdiamondheartmaltesecrossdaggerdoubledaggercheckmarkballotcrossmusicalsharpmusicalflatmalesymbolfemalesymboltelephonetelephonerecorderphonographcopyrightcaretsinglelowquotemarkdoublelowquotemarkcursorleftcaretrightcaretdowncaretupcaretoverbardowntackupshoedownstileunderbarjotquaduptackcircleupstiledownshoerightshoeleftshoelefttackrighttackhebrew_doublelowlinehebrew_alephhebrew_bethebrew_bethhebrew_gimelhebrew_gimmelhebrew_dalethebrew_dalethhebrew_hehebrew_wawhebrew_zainhebrew_zayinhebrew_chethebrew_hethebrew_tethebrew_tethhebrew_yodhebrew_finalkaphhebrew_kaphhebrew_lamedhebrew_finalmemhebrew_memhebrew_finalnunhebrew_nunhebrew_samechhebrew_samekhhebrew_ayinhebrew_finalpehebrew_pehebrew_finalzadehebrew_finalzadihebrew_zadehebrew_zadihebrew_qophhebrew_kufhebrew_reshhebrew_shinhebrew_tawhebrew_tafThai_kokaiThai_khokhaiThai_khokhuatThai_khokhwaiThai_khokhonThai_khorakhangThai_ngonguThai_chochanThai_chochingThai_chochangThai_sosoThai_chochoeThai_yoyingThai_dochadaThai_topatakThai_thothanThai_thonangmonthoThai_thophuthaoThai_nonenThai_dodekThai_totaoThai_thothungThai_thothahanThai_thothongThai_nonuThai_bobaimaiThai_poplaThai_phophungThai_fofaThai_phophanThai_fofanThai_phosamphaoThai_momaThai_yoyakThai_roruaThai_ruThai_lolingThai_luThai_wowaenThai_sosalaThai_sorusiThai_sosuaThai_hohipThai_lochulaThai_oangThai_honokhukThai_paiyannoiThai_saraaThai_maihanakatThai_saraaaThai_saraamThai_saraiThai_saraiiThai_saraueThai_saraueeThai_sarauThai_sarauuThai_phinthuThai_maihanakat_maithoThai_bahtThai_saraeThai_saraaeThai_saraoThai_saraaimaimuanThai_saraaimaimalaiThai_lakkhangyaoThai_maiyamokThai_maitaikhuThai_maiekThai_maithoThai_maitriThai_maichattawaThai_thanthakhatThai_nikhahitThai_leksunThai_leknungThai_leksongThai_leksamThai_leksiThai_lekhaThai_lekhokThai_lekchetThai_lekpaetThai_lekkaoHangul_KiyeogHangul_SsangKiyeogHangul_KiyeogSiosHangul_NieunHangul_NieunJieujHangul_NieunHieuhHangul_DikeudHangul_SsangDikeudHangul_RieulHangul_RieulKiyeogHangul_RieulMieumHangul_RieulPieubHangul_RieulSiosHangul_RieulTieutHangul_RieulPhieufHangul_RieulHieuhHangul_MieumHangul_PieubHangul_SsangPieubHangul_PieubSiosHangul_SiosHangul_SsangSiosHangul_IeungHangul_JieujHangul_SsangJieujHangul_CieucHangul_KhieuqHangul_TieutHangul_PhieufHangul_HieuhHangul_AHangul_AEHangul_YAHangul_YAEHangul_EOHangul_EHangul_YEOHangul_YEHangul_OHangul_WAHangul_WAEHangul_OEHangul_YOHangul_UHangul_WEOHangul_WEHangul_WIHangul_YUHangul_EUHangul_YIHangul_IHangul_J_KiyeogHangul_J_SsangKiyeogHangul_J_KiyeogSiosHangul_J_NieunHangul_J_NieunJieujHangul_J_NieunHieuhHangul_J_DikeudHangul_J_RieulHangul_J_RieulKiyeogHangul_J_RieulMieumHangul_J_RieulPieubHangul_J_RieulSiosHangul_J_RieulTieutHangul_J_RieulPhieufHangul_J_RieulHieuhHangul_J_MieumHangul_J_PieubHangul_J_PieubSiosHangul_J_SiosHangul_J_SsangSiosHangul_J_IeungHangul_J_JieujHangul_J_CieucHangul_J_KhieuqHangul_J_TieutHangul_J_PhieufHangul_J_HieuhHangul_RieulYeorinHieuhHangul_SunkyeongeumMieumHangul_SunkyeongeumPieubHangul_PanSiosHangul_KkogjiDalrinIeungHangul_SunkyeongeumPhieufHangul_YeorinHieuhHangul_AraeAHangul_AraeAEHangul_J_PanSiosHangul_J_KkogjiDalrinIeungHangul_J_YeorinHieuhKorean_WonOEoeYdiaeresisEuroSign3270_Duplicate3270_FieldMark3270_Right23270_Left23270_BackTab3270_EraseEOF3270_EraseInput3270_Reset3270_Quit3270_PA13270_PA23270_PA33270_Test3270_Attn3270_CursorBlink3270_AltCursor3270_KeyClick3270_Jump3270_Ident3270_Rule3270_Copy3270_Play3270_Setup3270_Record3270_ChangeScreen3270_DeleteWord3270_ExSelect3270_CursorSelect3270_PrintScreen3270_EnterISO_LockISO_Level2_LatchISO_Level3_ShiftISO_Level3_LatchISO_Level3_LockISO_Group_LatchISO_Group_LockISO_Next_GroupISO_Next_Group_LockISO_Prev_GroupISO_Prev_Group_LockISO_First_GroupISO_First_Group_LockISO_Last_GroupISO_Last_Group_LockISO_Level5_ShiftISO_Level5_LatchISO_Level5_LockISO_Left_TabISO_Move_Line_UpISO_Move_Line_DownISO_Partial_Line_UpISO_Partial_Line_DownISO_Partial_Space_LeftISO_Partial_Space_RightISO_Set_Margin_LeftISO_Set_Margin_RightISO_Release_Margin_LeftISO_Release_Margin_RightISO_Release_Both_MarginsISO_Fast_Cursor_LeftISO_Fast_Cursor_RightISO_Fast_Cursor_UpISO_Fast_Cursor_DownISO_Continuous_UnderlineISO_Discontinuous_UnderlineISO_EmphasizeISO_Center_ObjectISO_Enterdead_gravedead_acutedead_circumflexdead_tildedead_perispomenidead_macrondead_brevedead_abovedotdead_diaeresisdead_aboveringdead_doubleacutedead_carondead_cedilladead_ogonekdead_iotadead_voiced_sounddead_semivoiced_sounddead_belowdotdead_hookdead_horndead_strokedead_abovecommadead_psilidead_abovereversedcommadead_dasiadead_doublegravedead_belowringdead_belowmacrondead_belowcircumflexdead_belowtildedead_belowbrevedead_belowdiaeresisdead_invertedbrevedead_belowcommadead_currencyAccessX_EnableAccessX_Feedback_EnableRepeatKeys_EnableSlowKeys_EnableBounceKeys_EnableStickyKeys_EnableMouseKeys_EnableMouseKeys_Accel_EnableOverlay1_EnableOverlay2_EnableAudibleBell_Enabledead_adead_Adead_edead_Edead_idead_Idead_odead_Odead_udead_Udead_small_schwadead_schwadead_capital_schwadead_SCHWAdead_greekdead_hamzadead_apostrophedead_lowlinedead_aboveverticallinedead_belowverticallinedead_longsolidusoverlaychChCHc_hC_hC_HFirst_Virtual_ScreenPrev_Virtual_ScreenNext_Virtual_ScreenLast_Virtual_ScreenTerminate_ServerPointer_LeftPointer_RightPointer_UpPointer_DownPointer_UpLeftPointer_UpRightPointer_DownLeftPointer_DownRightPointer_Button_DfltPointer_Button1Pointer_Button2Pointer_Button3Pointer_Button4Pointer_Button5Pointer_DblClick_DfltPointer_DblClick1Pointer_DblClick2Pointer_DblClick3Pointer_DblClick4Pointer_DblClick5Pointer_Drag_DfltPointer_Drag1Pointer_Drag2Pointer_Drag3Pointer_Drag4Pointer_EnableKeysPointer_AcceleratePointer_DfltBtnNextPointer_DfltBtnPrevPointer_Drag5BackSpaceTabLinefeedClearReturnPauseScroll_LockSys_ReqEscapeMulti_keySunComposeKanjiMuhenkanHenkan_ModeHenkanRomajiHiraganaKatakanaHiragana_KatakanaZenkakuHankakuZenkaku_HankakuTourokuMassyoKana_LockKana_ShiftEisu_ShiftEisu_toggleHangulHangul_StartHangul_EndHangul_HanjaHangul_JamoHangul_RomajaCodeinputKanji_BangouHangul_CodeinputHangul_JeonjaHangul_BanjaHangul_PreHanjaHangul_PostHanjaSingleCandidateHangul_SingleCandidateMultipleCandidateZen_KohoHangul_MultipleCandidatePreviousCandidateMae_KohoHangul_PreviousCandidateHangul_SpecialHomeLeftUpRightDownPriorPage_UpSunPageUpNextPage_DownSunPageDownEndBeginSelectPrintSunPrint_ScreenExecuteInsertUndoSunUndoRedoSunAgainMenuFindSunFindCancelSunStopHelpBreakISO_Group_ShiftMode_switchscript_switchkana_switchArabic_switchGreek_switchHebrew_switchHangul_switchSunAltGraphNum_LockKP_SpaceKP_TabKP_EnterKP_F1KP_F2KP_F3KP_F4KP_HomeKP_LeftKP_UpKP_RightKP_DownKP_PriorKP_Page_UpKP_NextKP_Page_DownKP_EndKP_BeginKP_InsertKP_DeleteKP_MultiplyKP_AddKP_SeparatorKP_SubtractKP_DecimalKP_DivideKP_0KP_1KP_2KP_3KP_4KP_5KP_6KP_7KP_8KP_9KP_EqualF1F2F3F4F5F6F7F8F9F10F11L1F12L2F13L3F14L4F15L5F16L6F17L7F18L8F19L9F20L10F21R1F22R2F23R3F24R4F25R5F26R6F27R7F28R8F29R9F30R10F31R11F32R12F33R13F34R14F35R15Shift_LShift_RControl_LControl_RCaps_LockShift_LockMeta_LMeta_RAlt_LAlt_RSuper_LSuper_RHyper_LHyper_Rbraille_dot_1braille_dot_2braille_dot_3braille_dot_4braille_dot_5braille_dot_6braille_dot_7braille_dot_8braille_dot_9braille_dot_10DeleteVoidSymbolIbreveibreveWcircumflexwcircumflexYcircumflexycircumflexSCHWAObarredOhornohornUhornuhornZstrokezstrokeEZHOcaronocaronGcarongcaronschwaobarredezhcombining_gravecombining_acutecombining_tildecombining_hookcombining_belowdotCyrillic_GHE_barCyrillic_ghe_barCyrillic_ZHE_descenderCyrillic_zhe_descenderCyrillic_KA_descenderCyrillic_ka_descenderCyrillic_KA_vertstrokeCyrillic_ka_vertstrokeCyrillic_EN_descenderCyrillic_en_descenderCyrillic_U_straightCyrillic_u_straightCyrillic_U_straight_barCyrillic_u_straight_barCyrillic_HA_descenderCyrillic_ha_descenderCyrillic_CHE_descenderCyrillic_che_descenderCyrillic_CHE_vertstrokeCyrillic_che_vertstrokeCyrillic_SHHACyrillic_shhaCyrillic_SCHWACyrillic_schwaCyrillic_I_macronCyrillic_i_macronCyrillic_O_barCyrillic_o_barCyrillic_U_macronCyrillic_u_macronArmenian_AYBArmenian_BENArmenian_GIMArmenian_DAArmenian_YECHArmenian_ZAArmenian_EArmenian_ATArmenian_TOArmenian_ZHEArmenian_INIArmenian_LYUNArmenian_KHEArmenian_TSAArmenian_KENArmenian_HOArmenian_DZAArmenian_GHATArmenian_TCHEArmenian_MENArmenian_HIArmenian_NUArmenian_SHAArmenian_VOArmenian_CHAArmenian_PEArmenian_JEArmenian_RAArmenian_SEArmenian_VEVArmenian_TYUNArmenian_REArmenian_TSOArmenian_VYUNArmenian_PYURArmenian_KEArmenian_OArmenian_FEArmenian_apostropheArmenian_accentArmenian_sheshtArmenian_exclamArmenian_amanakArmenian_separation_markArmenian_butArmenian_questionArmenian_paruykArmenian_aybArmenian_benArmenian_gimArmenian_daArmenian_yechArmenian_zaArmenian_eArmenian_atArmenian_toArmenian_zheArmenian_iniArmenian_lyunArmenian_kheArmenian_tsaArmenian_kenArmenian_hoArmenian_dzaArmenian_ghatArmenian_tcheArmenian_menArmenian_hiArmenian_nuArmenian_shaArmenian_voArmenian_chaArmenian_peArmenian_jeArmenian_raArmenian_seArmenian_vevArmenian_tyunArmenian_reArmenian_tsoArmenian_vyunArmenian_pyurArmenian_keArmenian_oArmenian_feArmenian_ligature_ewArmenian_full_stopArmenian_verjaketArmenian_hyphenArmenian_yentamnaArabic_madda_aboveArabic_hamza_aboveArabic_hamza_belowArabic_0Arabic_1Arabic_2Arabic_3Arabic_4Arabic_5Arabic_6Arabic_7Arabic_8Arabic_9Arabic_percentArabic_superscript_alefArabic_ttehArabic_pehArabic_tchehArabic_ddalArabic_rrehArabic_jehArabic_vehArabic_kehehArabic_gafArabic_noon_ghunnaArabic_heh_doachashmeeArabic_heh_goalFarsi_yehArabic_farsi_yehArabic_yeh_bareeArabic_fullstopFarsi_0Farsi_1Farsi_2Farsi_3Farsi_4Farsi_5Farsi_6Farsi_7Farsi_8Farsi_9Sinh_ngSinh_h2Sinh_aSinh_aaSinh_aeSinh_aeeSinh_iSinh_iiSinh_uSinh_uuSinh_riSinh_riiSinh_luSinh_luuSinh_eSinh_eeSinh_aiSinh_oSinh_ooSinh_auSinh_kaSinh_khaSinh_gaSinh_ghaSinh_ng2Sinh_ngaSinh_caSinh_chaSinh_jaSinh_jhaSinh_nyaSinh_jnyaSinh_njaSinh_ttaSinh_tthaSinh_ddaSinh_ddhaSinh_nnaSinh_nddaSinh_thaSinh_thhaSinh_dhaSinh_dhhaSinh_naSinh_ndhaSinh_paSinh_phaSinh_baSinh_bhaSinh_maSinh_mbaSinh_yaSinh_raSinh_laSinh_vaSinh_shaSinh_sshaSinh_saSinh_haSinh_llaSinh_faSinh_alSinh_aa2Sinh_ae2Sinh_aee2Sinh_i2Sinh_ii2Sinh_u2Sinh_uu2Sinh_ru2Sinh_e2Sinh_ee2Sinh_ai2Sinh_o2Sinh_oo2Sinh_au2Sinh_lu2Sinh_ruu2Sinh_luu2Sinh_kunddaliyaGeorgian_anGeorgian_banGeorgian_ganGeorgian_donGeorgian_enGeorgian_vinGeorgian_zenGeorgian_tanGeorgian_inGeorgian_kanGeorgian_lasGeorgian_manGeorgian_narGeorgian_onGeorgian_parGeorgian_zharGeorgian_raeGeorgian_sanGeorgian_tarGeorgian_unGeorgian_pharGeorgian_kharGeorgian_ghanGeorgian_qarGeorgian_shinGeorgian_chinGeorgian_canGeorgian_jilGeorgian_cilGeorgian_charGeorgian_xanGeorgian_jhanGeorgian_haeGeorgian_heGeorgian_hieGeorgian_weGeorgian_harGeorgian_hoeGeorgian_fiBabovedotbabovedotDabovedotdabovedotFabovedotfabovedotLbelowdotlbelowdotMabovedotmabovedotPabovedotpabovedotSabovedotsabovedotTabovedottabovedotWgravewgraveWacutewacuteWdiaeresiswdiaeresisXabovedotxabovedotSSHARPAbelowdotabelowdotAhookahookAcircumflexacuteacircumflexacuteAcircumflexgraveacircumflexgraveAcircumflexhookacircumflexhookAcircumflextildeacircumflextildeAcircumflexbelowdotacircumflexbelowdotAbreveacuteabreveacuteAbrevegraveabrevegraveAbrevehookabrevehookAbrevetildeabrevetildeAbrevebelowdotabrevebelowdotEbelowdotebelowdotEhookehookEtildeetildeEcircumflexacuteecircumflexacuteEcircumflexgraveecircumflexgraveEcircumflexhookecircumflexhookEcircumflextildeecircumflextildeEcircumflexbelowdotecircumflexbelowdotIhookihookIbelowdotibelowdotObelowdotobelowdotOhookohookOcircumflexacuteocircumflexacuteOcircumflexgraveocircumflexgraveOcircumflexhookocircumflexhookOcircumflextildeocircumflextildeOcircumflexbelowdotocircumflexbelowdotOhornacuteohornacuteOhorngraveohorngraveOhornhookohornhookOhorntildeohorntildeOhornbelowdotohornbelowdotUbelowdotubelowdotUhookuhookUhornacuteuhornacuteUhorngraveuhorngraveUhornhookuhornhookUhorntildeuhorntildeUhornbelowdotuhornbelowdotYgraveygraveYbelowdotybelowdotYhookyhookYtildeytildeleftsingleanglequotemarkrightsingleanglequotemarkzerosuperiorfoursuperiorfivesuperiorsixsuperiorsevensuperioreightsuperiorninesuperiorzerosubscriptonesubscripttwosubscriptthreesubscriptfoursubscriptfivesubscriptsixsubscriptsevensubscripteightsubscriptninesubscriptEcuSignColonSignCruzeiroSignFFrancSignLiraSignMillSignNairaSignPesetaSignRupeeSignWonSignNewSheqelSignDongSignpartdifferentialemptysetelementofnotelementofcontainsassquarerootcuberootfourthrootdintegraltintegralbecausenotapproxeqapproxeqnotidenticalstricteqbraille_blankbraille_dots_1braille_dots_2braille_dots_12braille_dots_3braille_dots_13braille_dots_23braille_dots_123braille_dots_4braille_dots_14braille_dots_24braille_dots_124braille_dots_34braille_dots_134braille_dots_234braille_dots_1234braille_dots_5braille_dots_15braille_dots_25braille_dots_125braille_dots_35braille_dots_135braille_dots_235braille_dots_1235braille_dots_45braille_dots_145braille_dots_245braille_dots_1245braille_dots_345braille_dots_1345braille_dots_2345braille_dots_12345braille_dots_6braille_dots_16braille_dots_26braille_dots_126braille_dots_36braille_dots_136braille_dots_236braille_dots_1236braille_dots_46braille_dots_146braille_dots_246braille_dots_1246braille_dots_346braille_dots_1346braille_dots_2346braille_dots_12346braille_dots_56braille_dots_156braille_dots_256braille_dots_1256braille_dots_356braille_dots_1356braille_dots_2356braille_dots_12356braille_dots_456braille_dots_1456braille_dots_2456braille_dots_12456braille_dots_3456braille_dots_13456braille_dots_23456braille_dots_123456braille_dots_7braille_dots_17braille_dots_27braille_dots_127braille_dots_37braille_dots_137braille_dots_237braille_dots_1237braille_dots_47braille_dots_147braille_dots_247braille_dots_1247braille_dots_347braille_dots_1347braille_dots_2347braille_dots_12347braille_dots_57braille_dots_157braille_dots_257braille_dots_1257braille_dots_357braille_dots_1357braille_dots_2357braille_dots_12357braille_dots_457braille_dots_1457braille_dots_2457braille_dots_12457braille_dots_3457braille_dots_13457braille_dots_23457braille_dots_123457braille_dots_67braille_dots_167braille_dots_267braille_dots_1267braille_dots_367braille_dots_1367braille_dots_2367braille_dots_12367braille_dots_467braille_dots_1467braille_dots_2467braille_dots_12467braille_dots_3467braille_dots_13467braille_dots_23467braille_dots_123467braille_dots_567braille_dots_1567braille_dots_2567braille_dots_12567braille_dots_3567braille_dots_13567braille_dots_23567braille_dots_123567braille_dots_4567braille_dots_14567braille_dots_24567braille_dots_124567braille_dots_34567braille_dots_134567braille_dots_234567braille_dots_1234567braille_dots_8braille_dots_18braille_dots_28braille_dots_128braille_dots_38braille_dots_138braille_dots_238braille_dots_1238braille_dots_48braille_dots_148braille_dots_248braille_dots_1248braille_dots_348braille_dots_1348braille_dots_2348braille_dots_12348braille_dots_58braille_dots_158braille_dots_258braille_dots_1258braille_dots_358braille_dots_1358braille_dots_2358braille_dots_12358braille_dots_458braille_dots_1458braille_dots_2458braille_dots_12458braille_dots_3458braille_dots_13458braille_dots_23458braille_dots_123458braille_dots_68braille_dots_168braille_dots_268braille_dots_1268braille_dots_368braille_dots_1368braille_dots_2368braille_dots_12368braille_dots_468braille_dots_1468braille_dots_2468braille_dots_12468braille_dots_3468braille_dots_13468braille_dots_23468braille_dots_123468braille_dots_568braille_dots_1568braille_dots_2568braille_dots_12568braille_dots_3568braille_dots_13568braille_dots_23568braille_dots_123568braille_dots_4568braille_dots_14568braille_dots_24568braille_dots_124568braille_dots_34568braille_dots_134568braille_dots_234568braille_dots_1234568braille_dots_78braille_dots_178braille_dots_278braille_dots_1278braille_dots_378braille_dots_1378braille_dots_2378braille_dots_12378braille_dots_478braille_dots_1478braille_dots_2478braille_dots_12478braille_dots_3478braille_dots_13478braille_dots_23478braille_dots_123478braille_dots_578braille_dots_1578braille_dots_2578braille_dots_12578braille_dots_3578braille_dots_13578braille_dots_23578braille_dots_123578braille_dots_4578braille_dots_14578braille_dots_24578braille_dots_124578braille_dots_34578braille_dots_134578braille_dots_234578braille_dots_1234578braille_dots_678braille_dots_1678braille_dots_2678braille_dots_12678braille_dots_3678braille_dots_13678braille_dots_23678braille_dots_123678braille_dots_4678braille_dots_14678braille_dots_24678braille_dots_124678braille_dots_34678braille_dots_134678braille_dots_234678braille_dots_1234678braille_dots_5678braille_dots_15678braille_dots_25678braille_dots_125678braille_dots_35678braille_dots_135678braille_dots_235678braille_dots_1235678braille_dots_45678braille_dots_145678braille_dots_245678braille_dots_1245678braille_dots_345678braille_dots_1345678braille_dots_2345678braille_dots_12345678hpmute_acutemute_acutehpmute_gravemute_gravehpmute_asciicircummute_asciicircumhpmute_diaeresismute_diaeresishpmute_asciitildemute_asciitildehpliralirahpguilderguilderhpYdiaeresishpIOIOhplongminuslongminushpblockblockDdiaeresisDacute_accentDcedilla_accentDcircumflex_accentDgrave_accentDtildeDring_accentDRemovehpModelock1hpModelock2hpResetResethpSystemSystemhpUserUserhpClearLineClearLinehpInsertLineInsertLinehpDeleteLineDeleteLinehpInsertCharInsertCharhpDeleteCharDeleteCharhpBackTabBackTabhpKP_BackTabKP_BackTabExt16bit_LExt16bit_RosfCopyosfCutosfPasteosfBackTabosfBackSpaceosfClearosfEscapeosfAddModeosfPrimaryPasteosfQuickPasteosfPageLeftosfPageUposfPageDownosfPageRightosfActivateosfMenuBarosfLeftosfUposfRightosfDownosfEndLineosfBeginLineosfEndDataosfBeginDataosfPrevMenuosfNextMenuosfPrevFieldosfNextFieldosfSelectosfInsertosfUndoosfMenuosfCancelosfHelposfSelectAllosfDeselectAllosfReselectosfExtendosfRestoreosfDeleteSunFA_GraveSunFA_CircumSunFA_TildeSunFA_AcuteSunFA_DiaeresisSunFA_CedillaSunF36SunF37SunSys_ReqSunPropsSunFrontSunCopySunOpenSunPasteSunCutSunPowerSwitchSunAudioLowerVolumeSunAudioMuteSunAudioRaiseVolumeSunVideoDegaussSunVideoLowerBrightnessSunVideoRaiseBrightnessSunPowerSwitchShiftXF86MediaPlayPauseXF86ExitXF86AudioBassBoostXF86SportXF86BrightnessAutoXF86MonBrightnessAutoXF86DisplayOffXF86OKXF86GoToXF86InfoXF86VendorLogoXF86MediaSelectProgramGuideXF86MediaSelectHomeXF86MediaLanguageMenuXF86MediaTitleMenuXF86AudioChannelModeXF86AspectRatioXF86MediaSelectPCXF86MediaSelectTVXF86MediaSelectCableXF86MediaSelectVCRXF86MediaSelectVCRPlusXF86MediaSelectSatelliteXF86MediaSelectTapeXF86MediaSelectRadioXF86MediaSelectTunerXF86MediaPlayerXF86MediaSelectTeletextXF86DVDXF86MediaSelectDVDXF86MediaSelectAuxiliaryXF86AudioXF86ChannelUpXF86ChannelDownXF86MediaPlaySlowXF86BreakXF86NumberEntryModeXF86VideoPhoneXF86ZoomResetXF86EditorXF86GraphicsEditorXF86PresentationXF86DatabaseXF86VoicemailXF86AddressbookXF86DisplayToggleXF86SpellCheckXF86ContextMenuXF86MediaRepeatXF8610ChannelsUpXF8610ChannelsDownXF86ImagesXF86NotificationCenterXF86PickupPhoneXF86HangupPhoneXF86LinkPhoneXF86FnXF86Fn_EscXF86Fn_F1XF86Fn_F2XF86Fn_F3XF86Fn_F4XF86Fn_F5XF86Fn_F6XF86Fn_F7XF86Fn_F8XF86Fn_F9XF86Fn_F10XF86Fn_F11XF86Fn_F12XF86Fn_1XF86Fn_2XF86Fn_DXF86Fn_EXF86Fn_FXF86Fn_SXF86Fn_BXF86FnRightShiftXF86Numeric0XF86Numeric1XF86Numeric2XF86Numeric3XF86Numeric4XF86Numeric5XF86Numeric6XF86Numeric7XF86Numeric8XF86Numeric9XF86NumericStarXF86NumericPoundXF86NumericAXF86NumericBXF86NumericCXF86NumericDXF86CameraFocusXF86WPSButtonXF86CameraZoomInXF86CameraZoomOutXF86CameraUpXF86CameraDownXF86CameraLeftXF86CameraRightXF86AttendantOnXF86AttendantOffXF86AttendantToggleXF86LightsToggleXF86ALSToggleXF86RefreshRateToggleXF86ButtonconfigXF86TaskmanagerXF86JournalXF86ControlPanelXF86AppSelectXF86ScreensaverXF86VoiceCommandXF86AssistantXF86EmojiPickerXF86DictateXF86CameraAccessEnableXF86CameraAccessDisableXF86CameraAccessToggleXF86AccessibilityXF86DoNotDisturbXF86BrightnessMinXF86BrightnessMaxXF86ElectronicPrivacyScreenOnXF86ElectronicPrivacyScreenOffXF86ActionOnSelectionXF86ContextualInsertXF86ContextualQueryXF86KbdInputAssistPrevXF86KbdInputAssistNextXF86KbdInputAssistPrevgroupXF86KbdInputAssistNextgroupXF86KbdInputAssistAcceptXF86KbdInputAssistCancelXF86RightUpXF86RightDownXF86LeftUpXF86LeftDownXF86RootMenuXF86MediaTopMenuXF86Numeric11XF86Numeric12XF86AudioDescXF863DModeXF86NextFavoriteXF86StopRecordXF86PauseRecordXF86VODXF86UnmuteXF86FastReverseXF86SlowReverseXF86DataXF86OnScreenKeyboardXF86PrivacyScreenToggleXF86SelectiveScreenshotXF86NextElementXF86PreviousElementXF86AutopilotEngageToggleXF86MarkWaypointXF86SosXF86NavChartXF86FishingChartXF86SingleRangeRadarXF86DualRangeRadarXF86RadarOverlayXF86TraditionalSonarXF86ClearvuSonarXF86SidevuSonarXF86NavInfoXF86Macro1XF86Macro2XF86Macro3XF86Macro4XF86Macro5XF86Macro6XF86Macro7XF86Macro8XF86Macro9XF86Macro10XF86Macro11XF86Macro12XF86Macro13XF86Macro14XF86Macro15XF86Macro16XF86Macro17XF86Macro18XF86Macro19XF86Macro20XF86Macro21XF86Macro22XF86Macro23XF86Macro24XF86Macro25XF86Macro26XF86Macro27XF86Macro28XF86Macro29XF86Macro30XF86MacroRecordStartXF86MacroRecordStopXF86MacroPresetCycleXF86MacroPreset1XF86MacroPreset2XF86MacroPreset3XF86KbdLcdMenu1XF86KbdLcdMenu2XF86KbdLcdMenu3XF86KbdLcdMenu4XF86KbdLcdMenu5XF86PerformanceModeXF86Switch_VT_1XF86Switch_VT_2XF86Switch_VT_3XF86Switch_VT_4XF86Switch_VT_5XF86Switch_VT_6XF86Switch_VT_7XF86Switch_VT_8XF86Switch_VT_9XF86Switch_VT_10XF86Switch_VT_11XF86Switch_VT_12XF86UngrabXF86ClearGrabXF86Next_VModeXF86Prev_VModeXF86LogWindowTreeXF86LogGrabInfoXF86ModeLockXF86MonBrightnessUpXF86MonBrightnessDownXF86KbdLightOnOffXF86KbdBrightnessUpXF86KbdBrightnessDownXF86MonBrightnessCycleXF86StandbyXF86AudioLowerVolumeXF86AudioMuteXF86AudioRaiseVolumeXF86AudioPlayXF86AudioStopXF86AudioPrevXF86AudioNextXF86HomePageXF86MailXF86StartXF86SearchXF86AudioRecordXF86CalculatorXF86MemoXF86ToDoListXF86CalendarXF86PowerDownXF86ContrastAdjustXF86RockerUpXF86RockerDownXF86RockerEnterXF86BackXF86ForwardXF86StopXF86RefreshXF86PowerOffXF86WakeUpXF86EjectXF86ScreenSaverXF86WWWXF86SleepXF86FavoritesXF86AudioPauseXF86AudioMediaXF86MyComputerXF86VendorHomeXF86LightBulbXF86ShopXF86HistoryXF86OpenURLXF86AddFavoriteXF86HotLinksXF86BrightnessAdjustXF86FinanceXF86CommunityXF86AudioRewindXF86BackForwardXF86Launch0XF86Launch1XF86Launch2XF86Launch3XF86Launch4XF86Launch5XF86Launch6XF86Launch7XF86Launch8XF86Launch9XF86LaunchAXF86LaunchBXF86LaunchCXF86LaunchDXF86LaunchEXF86LaunchFXF86ApplicationLeftXF86ApplicationRightXF86BookXF86CDXF86MediaSelectCDXF86CalculaterXF86ClearXF86CloseXF86CopyXF86CutXF86DisplayXF86DOSXF86DocumentsXF86ExcelXF86ExplorerXF86GameXF86GoXF86iTouchXF86LogOffXF86MarketXF86MeetingXF86MenuKBXF86MenuPBXF86MySitesXF86NewXF86NewsXF86OfficeHomeXF86OpenXF86OptionXF86PasteXF86PhoneXF86QXF86ReplyXF86ReloadXF86RotateWindowsXF86RotationPBXF86RotationKBXF86SaveXF86ScrollUpXF86ScrollDownXF86ScrollClickXF86SendXF86SpellXF86SplitScreenXF86SupportXF86TaskPaneXF86TerminalXF86ToolsXF86TravelXF86UserPBXF86User1KBXF86User2KBXF86VideoXF86WheelButtonXF86WordXF86XferXF86ZoomInXF86ZoomOutXF86AwayXF86MessengerXF86WebCamXF86MailForwardXF86PicturesXF86MusicXF86BatteryXF86BluetoothXF86WLANXF86UWBXF86AudioForwardXF86AudioRepeatXF86AudioRandomPlayXF86SubtitleXF86AudioCycleTrackXF86CycleAngleXF86FrameBackXF86FrameForwardXF86TimeXF86SelectXF86ViewXF86TopMenuXF86RedXF86GreenXF86YellowXF86BlueXF86SuspendXF86HibernateXF86TouchpadToggleXF86TouchpadOnXF86TouchpadOffXF86AudioMicMuteXF86KeyboardXF86WWANXF86RFKillXF86AudioPresetXF86RotationLockToggleXF86FullScreen";

pub(super) static KEYSYM_TO_IDX: PhfMap<u32, u16> = PhfMap {
    key: 10656210946825422025,
    disps: &[(1, 2), (1, 461), (1, 6), (0, 0), (0, 105), (0, 21), (0, 262), (0, 1), (1, 24), (0, 2), (1, 44), (0, 164), (0, 6), (0, 520), (0, 26), (0, 101), (0, 30), (0, 276), (0, 0), (0, 571), (0, 136), (0, 11), (0, 9), (0, 6), (1, 2), (0, 1), (0, 40), (1, 13), (0, 15), (0, 28), (0, 258), (0, 11), (0, 27), (0, 471), (0, 196), (0, 64), (0, 363), (0, 289), (0, 5), (0, 1258), (0, 11), (1, 224), (0, 1235), (0, 1037), (0, 2439), (0, 333), (0, 1530), (0, 854), (0, 3), (1, 4), (0, 24), (0, 6), (1, 6), (0, 3), (1, 450), (0, 1865), (0, 3), (0, 216), (0, 56), (0, 2077), (0, 127), (0, 268), (0, 21), (0, 7), (0, 521), (0, 27), (0, 330), (0, 21), (0, 33), (0, 278), (0, 6), (0, 304), (3, 317), (0, 118), (0, 6), (0, 427), (0, 13), (0, 0), (0, 1761), (0, 5), (0, 2), (3, 365), (0, 6), (0, 29), (0, 0), (0, 0), (0, 64), (0, 58), (0, 139), (0, 633), (0, 217), (0, 2), (0, 14), (0, 8), (0, 657), (0, 6), (0, 27), (0, 88), (0, 1), (0, 53), (0, 9), (0, 1), (0, 882), (0, 83), (0, 15), (0, 7), (0, 62), (0, 10), (0, 317), (0, 66), (0, 54), (1, 199), (0, 1182), (0, 2), (0, 11), (0, 1), (0, 32), (0, 21), (0, 2), (0, 28), (0, 12), (0, 0), (0, 2), (0, 2), (0, 7), (0, 29), (0, 0), (0, 50), (0, 20), (0, 14), (0, 86), (0, 98), (0, 1), (0, 2), (1, 17), (1, 0), (0, 1), (1, 1), (0, 30), (0, 2), (0, 37), (0, 3), (0, 386), (0, 1019), (0, 20), (0, 0), (0, 55), (0, 0), (0, 165), (1, 1884), (0, 5), (0, 2321), (0, 150), (0, 6), (0, 1), (0, 5), (0, 8), (1, 1186), (0, 49), (0, 14), (0, 2093), (1, 6), (0, 117), (1, 152), (1, 2), (0, 2), (0, 75), (1, 0), (1, 8), (2, 26), (1, 189), (0, 830), (0, 1536), (0, 1), (1, 0), (1, 257), (0, 22), (0, 178), (1, 20), (1, 8), (0, 59), (0, 24), (1, 293), (0, 648), (0, 212), (0, 706), (0, 149), (0, 8), (0, 741), (0, 841), (0, 2), (0, 1), (0, 24), (0, 477), (1, 0), (0, 1), (0, 320), (0, 40), (1, 8), (0, 822), (0, 1), (0, 2208), (0, 4), (0, 1597), (0, 631), (1, 1121), (0, 5), (1, 632), (0, 377), (0, 5), (3, 1987), (0, 201), (0, 867), (1, 1314), (0, 252), (0, 1100), (0, 63), (0, 117), (0, 1158), (1, 20), (0, 25), (0, 1422), (0, 8), (1, 194), (2, 225), (1, 57), (0, 22), (2, 412), (0, 0), (1, 447), (1, 124), (2, 769), (0, 29), (0, 615), (0, 358), (0, 268), (0, 41), (0, 200), (0, 182), (0, 2), (1, 4), (0, 2), (0, 37), (2, 808), (0, 1421), (0, 24), (2, 2257), (0, 2), (0, 151), (1, 827), (0, 32), (4, 1401), (0, 17), (0, 30), (0, 84), (0, 3), (0, 2465), (0, 0), (0, 22), (0, 1442), (0, 6), (3, 31), (1, 1819), (0, 1), (0, 18), (10, 2142), (0, 19), (0, 10), (4, 1381), (0, 6), (0, 724), (0, 3), (0, 10), (0, 32), (0, 142), (0, 5), (0, 47), (0, 34), (0, 0), (0, 1116), (0, 104), (0, 421), (2, 468), (0, 19), (0, 120), (0, 166), (0, 2), (0, 78), (0, 399), (0, 0), (0, 22), (0, 36), (0, 0), (0, 33), (0, 10), (0, 4), (0, 4), (0, 4), (0, 12), (0, 0), (0, 6), (0, 6), (0, 4), (0, 4), (0, 277), (0, 3), (0, 2), (0, 49), (0, 469), (0, 27), (0, 269), (0, 276), (0, 878), (0, 221), (0, 583), (0, 0), (0, 627), (0, 13), (0, 0), (0, 682), (1, 1200), (0, 24), (2, 2224), (0, 1), (0, 310), (0, 330), (0, 0), (0, 1), (0, 1559), (0, 243), (0, 272), (0, 1712), (1, 51), (0, 1307), (1, 145), (0, 222), (0, 1652), (1, 5), (0, 0), (1, 352), (0, 489), (1, 25), (3, 1602), (0, 16), (1, 0), (3, 425), (0, 32), (2, 14), (0, 222), (0, 11), (1, 2), (1, 694), (0, 43), (1, 1556), (6, 434), (0, 66), (0, 225), (0, 141), (2, 1784), (0, 4), (0, 271), (1, 1734), (0, 1), (0, 1091), (0, 1683), (0, 0), (0, 753), (2, 1687), (0, 0), (4, 660), (1, 237), (0, 2), (0, 431), (1, 877), (0, 632), (4, 820), (0, 116), (12, 2312), (4, 471), (0, 10), (2, 853), (0, 296), (1, 187), (39, 8), (0, 100), (0, 511), (0, 2222), (18, 923), (1, 2), (3, 23), (0, 1), (1, 1), (0, 2131), (0, 1405), (0, 1007), (1, 461), (1, 482), (0, 144), (1, 2161), (0, 398), (5, 33), (0, 1158), (0, 1383), (0, 2274), (1, 148), (7, 2019), (1, 1030), (0, 328), (3, 1424), (0, 385), (1, 17), (0, 54), (7, 577), (0, 18), (22, 77), (32, 281), (1, 1425), (0, 2128), (52, 292), (0, 1225), (0, 8), (0, 477), (11, 1757), (0, 859), (0, 65), (0, 5), (0, 2), (1, 864), (0, 11), (0, 49), (8, 2148), (1, 130), (1, 823), (7, 72), (0, 12), (0, 1025), (0, 195), (0, 0), (0, 447), (0, 195), (0, 13), (0, 134), (0, 1), (2, 778), (0, 888), (0, 21), (0, 0), (3, 830), (1, 2), (0, 195), (0, 440), (40, 2388), (0, 39), (0, 59), (0, 4), (37, 1180), (0, 2), (0, 482), (0, 8), (0, 20), (0, 2), (0, 39), (0, 8), (0, 0), (0, 1), (0, 16), (0, 19), (0, 0), (1, 1475), (1, 800), (0, 61), (0, 302), (0, 513), (0, 4), (0, 451), (1, 3), (0, 1), (0, 50), (6, 612), (0, 73), (0, 0), (1, 6), (0, 12), (0, 25), (2, 1287), (0, 84), (0, 4), (54, 2090), (0, 1), (0, 0), (0, 1167), (0, 953), (4, 1286), (7, 1689), (0, 6), (0, 577), (3, 656), (0, 8), (0, 78), (8, 215), (0, 8), (6, 705), (1, 518), (1, 1), (36, 1646), (0, 18), (1, 7), (2, 1439), (0, 11), (1, 3), (21, 1666), (1, 4)],
    map: &[585, 1528, 660, 1724, 2266, 1808, 304, 2042, 201, 1625, 370, 1505, 620, 1802, 727, 77, 1484, 845, 373, 1619, 674, 2031, 1302, 1847, 2237, 2217, 1951, 1014, 782, 1204, 274, 899, 439, 2441, 541, 729, 862, 155, 2022, 1864, 2308, 1319, 1605, 1040, 2157, 926, 2448, 989, 2107, 42, 1054, 2480, 1894, 713, 1144, 892, 1721, 2033, 2479, 2040, 595, 2231, 908, 108, 1338, 1629, 1593, 1688, 708, 861, 210, 388, 754, 1749, 2582, 526, 224, 264, 1092, 647, 267, 778, 983, 2117, 2600, 697, 1943, 2430, 549, 237, 1777, 2407, 29, 2518, 918, 287, 2034, 2210, 2565, 2500, 920, 878, 1188, 104, 2389, 1493, 668, 644, 465, 748, 1267, 1598, 2350, 2509, 90, 1977, 816, 167, 2488, 1632, 406, 1628, 1086, 944, 2282, 2185, 1771, 284, 963, 2285, 235, 939, 652, 888, 1902, 2390, 1670, 344, 1502, 392, 1964, 1342, 588, 1448, 2528, 1995, 633, 209, 579, 1521, 1745, 1364, 288, 508, 2586, 312, 670, 812, 2120, 1648, 99, 2443, 1114, 1511, 428, 2625, 2190, 1237, 421, 1853, 849, 2536, 1839, 799, 1838, 502, 2555, 1295, 2618, 2224, 246, 1651, 1725, 195, 17, 149, 2373, 117, 649, 350, 765, 1729, 1602, 991, 1984, 221, 1170, 8, 2336, 1106, 1734, 707, 547, 773, 22, 947, 2371, 1850, 334, 1063, 1835, 2382, 2013, 2628, 1934, 1249, 976, 1445, 1194, 1681, 205, 653, 391, 198, 1072, 1871, 1795, 2470, 2475, 377, 451, 2090, 512, 1029, 1414, 161, 336, 53, 1623, 1844, 858, 397, 1799, 19, 188, 822, 1252, 1035, 897, 1116, 2203, 1577, 675, 2559, 141, 2622, 1467, 737, 1213, 1141, 530, 1675, 2571, 2150, 1960, 2001, 2461, 601, 440, 1422, 1120, 1529, 1714, 1867, 641, 358, 1940, 1687, 2482, 1032, 2361, 1308, 1523, 162, 592, 1520, 1069, 1895, 2460, 2108, 56, 1138, 1185, 1133, 2473, 2399, 1225, 2100, 2037, 217, 985, 1535, 671, 2297, 700, 2503, 432, 1952, 2064, 1578, 1394, 1889, 2471, 120, 994, 1460, 572, 2324, 536, 927, 1805, 2634, 859, 2241, 2604, 2376, 153, 2388, 1563, 1829, 2454, 2574, 1051, 2362, 2232, 1739, 497, 1215, 2014, 456, 1285, 2426, 83, 1001, 2290, 1978, 371, 2601, 1778, 2118, 1636, 921, 263, 1136, 1912, 1647, 1157, 865, 564, 2213, 2291, 2111, 1453, 2408, 1312, 1179, 1307, 1645, 687, 47, 1435, 1762, 1077, 1057, 1988, 409, 1360, 335, 353, 2027, 1620, 2607, 998, 853, 1957, 2322, 578, 696, 1168, 1131, 2116, 2055, 1352, 292, 2152, 593, 1087, 2580, 1452, 2419, 2552, 78, 1884, 725, 2249, 171, 717, 2029, 615, 1348, 1192, 1982, 2182, 1150, 2028, 363, 1925, 2481, 2061, 757, 91, 1788, 107, 528, 1, 521, 495, 2020, 2077, 2010, 2191, 1238, 686, 1205, 2494, 945, 2437, 635, 788, 883, 2436, 184, 1662, 1557, 2039, 1747, 1186, 704, 951, 2395, 2260, 2345, 208, 2561, 2605, 242, 2286, 556, 2327, 1300, 2384, 72, 2154, 1570, 2342, 1603, 1782, 1712, 1806, 1004, 689, 882, 1476, 643, 575, 433, 2431, 1265, 1743, 200, 724, 618, 2569, 1210, 1065, 1431, 1939, 1020, 96, 1379, 1872, 732, 1316, 313, 1125, 436, 384, 1012, 2570, 1764, 612, 1011, 1064, 135, 977, 1845, 2457, 446, 2502, 1554, 2267, 2421, 1180, 2623, 1800, 1078, 102, 1561, 1949, 1159, 1389, 2513, 1126, 640, 1737, 2306, 1486, 2365, 1117, 2611, 1039, 1473, 2141, 1187, 987, 1633, 1898, 1038, 648, 2204, 1914, 475, 952, 1052, 2485, 1571, 1668, 1412, 51, 402, 127, 1201, 286, 924, 1562, 912, 1074, 1677, 305, 1686, 1304, 231, 1975, 981, 1462, 513, 1540, 1666, 1231, 533, 36, 1585, 2255, 916, 1823, 752, 462, 2452, 2273, 2598, 1775, 412, 1480, 1430, 1046, 1455, 1860, 331, 1404, 826, 228, 2459, 389, 1119, 1286, 1508, 1828, 1694, 206, 2514, 1438, 684, 1969, 1222, 584, 28, 143, 1090, 1903, 1495, 2235, 1083, 1068, 733, 298, 1165, 307, 746, 103, 2143, 1855, 1756, 1118, 46, 233, 1318, 2214, 2176, 342, 225, 114, 787, 21, 2058, 1415, 1769, 2094, 656, 870, 1993, 485, 176, 1446, 2319, 2284, 2251, 1836, 2526, 2401, 1926, 1366, 999, 2579, 1793, 52, 936, 795, 2523, 203, 576, 426, 1323, 2418, 140, 1646, 2616, 81, 2248, 529, 15, 793, 1293, 1663, 1346, 190, 570, 880, 1494, 1753, 2132, 1510, 491, 367, 763, 2553, 2229, 192, 1041, 1596, 1987, 419, 1103, 624, 705, 2610, 152, 2295, 1787, 2367, 1999, 290, 948, 1485, 2375, 1183, 2070, 1313, 1443, 1942, 726, 1879, 974, 1716, 1061, 2468, 871, 1932, 293, 625, 2547, 2032, 2314, 2492, 1025, 1408, 2549, 1900, 1461, 148, 1376, 1679, 2076, 2109, 2025, 33, 1797, 1450, 1112, 2261, 789, 786, 70, 6, 2088, 848, 1958, 771, 404, 2572, 1400, 1746, 2216, 2304, 2592, 2455, 2489, 278, 1281, 2541, 2446, 324, 2495, 1842, 2155, 820, 2620, 621, 1385, 194, 369, 1049, 895, 1869, 1279, 783, 2287, 387, 1854, 182, 2587, 1560, 646, 1211, 1534, 599, 2444, 2564, 1410, 1044, 482, 1558, 39, 310, 1996, 1626, 930, 2103, 1699, 227, 476, 925, 1887, 275, 1703, 101, 2007, 1552, 2177, 1358, 1590, 226, 1624, 507, 1908, 1556, 2505, 959, 488, 2230, 97, 1487, 913, 489, 1007, 1976, 857, 2080, 1735, 2092, 2262, 1406, 1439, 1234, 1824, 1667, 851, 306, 437, 1748, 755, 1093, 179, 2599, 1776, 1634, 1336, 846, 65, 1548, 863, 239, 2065, 2068, 2053, 583, 2130, 1191, 483, 801, 1565, 2539, 1027, 1815, 1465, 841, 850, 2238, 413, 2511, 407, 784, 2483, 1447, 996, 967, 1097, 1573, 730, 1220, 619, 1770, 986, 1311, 129, 1599, 1837, 2110, 1673, 2524, 794, 1713, 780, 319, 889, 2533, 236, 2393, 2315, 2089, 2629, 1553, 1660, 1037, 1732, 1817, 2474, 1301, 520, 181, 2277, 694, 960, 115, 1630, 1148, 1591, 1708, 1862, 1113, 268, 1280, 429, 2377, 797, 553, 2347, 16, 420, 248, 2126, 747, 2134, 506, 1284, 1980, 2323, 1621, 2223, 919, 2603, 1383, 723, 125, 589, 2543, 68, 2379, 809, 1654, 2178, 1601, 2023, 2008, 299, 2047, 2188, 937, 1173, 2147, 2099, 2202, 1517, 2002, 1689, 1791, 1641, 968, 1516, 1876, 2429, 904, 431, 1937, 63, 1780, 543, 1416, 591, 58, 903, 844, 2337, 2069, 1870, 1717, 555, 975, 303, 434, 1425, 121, 2484, 361, 133, 1613, 400, 79, 1328, 94, 1115, 569, 1843, 664, 2072, 2621, 160, 20, 691, 1377, 1076, 2451, 1432, 1254, 1261, 881, 1208, 1674, 969, 673, 1199, 898, 523, 2051, 44, 1896, 1643, 600, 1798, 1417, 1424, 1036, 1378, 1618, 1878, 250, 552, 376, 1344, 2411, 511, 2205, 580, 1142, 609, 616, 2227, 1928, 1963, 906, 1050, 499, 1973, 1761, 1489, 473, 531, 735, 212, 2298, 1330, 2562, 1684, 1877, 2585, 1927, 979, 1583, 2258, 2534, 2567, 2222, 1030, 537, 2596, 230, 1498, 876, 1310, 545, 329, 2030, 459, 1392, 1305, 85, 501, 1356, 1840, 2097, 100, 2635, 50, 2159, 2349, 1321, 1003, 1298, 2593, 1858, 1018, 2391, 394, 2215, 1496, 1592, 2050, 1143, 940, 1913, 701, 220, 1081, 1614, 355, 2173, 240, 1228, 868, 1905, 2104, 1000, 2422, 1166, 1701, 191, 1466, 2212, 1644, 1478, 2383, 186, 1991, 2250, 2220, 645, 1710, 185, 1589, 87, 2012, 1289, 716, 424, 2283, 2123, 1649, 2397, 323, 695, 2105, 1110, 2386, 291, 1924, 561, 1291, 776, 214, 1066, 2148, 2551, 1171, 271, 2416, 417, 2352, 204, 891, 486, 1440, 962, 1399, 931, 1661, 1915, 2487, 1354, 1967, 810, 2428, 1492, 997, 1283, 1611, 1472, 1100, 703, 554, 1635, 2199, 1767, 1243, 71, 761, 2244, 658, 2608, 1059, 2544, 2062, 241, 1917, 972, 524, 2079, 1656, 2063, 1985, 568, 2380, 1883, 2149, 211, 1175, 1060, 2632, 146, 1946, 2128, 1997, 2163, 2310, 173, 105, 1023, 2136, 2195, 60, 657, 678, 1482, 829, 1315, 2225, 2263, 517, 1566, 744, 682, 1676, 941, 1419, 1209, 31, 518, 163, 2374, 1374, 966, 1875, 89, 1722, 174, 2240, 383, 261, 2516, 525, 1956, 2073, 1070, 2056, 1773, 54, 1154, 749, 1821, 2311, 2017, 965, 1706, 2348, 824, 245, 1856, 667, 1260, 607, 1433, 1129, 1950, 1697, 1203, 1736, 2309, 1457, 855, 2081, 1834, 990, 2236, 1390, 474, 1509, 48, 341, 1079, 2403, 382, 2576, 1639, 728, 1428, 37, 1811, 2207, 2289, 0, 1314, 1813, 1454, 1550, 311, 1411, 911, 1970, 2043, 326, 1638, 2228, 277, 1781, 923, 753, 2512, 1541, 815, 984, 2044, 772, 2275, 1091, 2316, 2597, 698, 1479, 463, 1911, 13, 917, 1671, 1584, 2520, 238, 2630, 177, 1774, 1122, 347, 1723, 2264, 1277, 2038, 1421, 2035, 683, 1650, 654, 1968, 2326, 1692, 1738, 1221, 2303, 1189, 151, 745, 2313, 1597, 2259, 1082, 2087, 1617, 2447, 943, 1370, 817, 2537, 877, 2591, 1322, 1818, 887, 2318, 1685, 2531, 1728, 301, 1768, 427, 59, 610, 2498, 914, 1658, 685, 1340, 659, 2472, 2542, 116, 2406, 955, 490, 1931, 2341, 2186, 2573, 357, 14, 480, 88, 1551, 1169, 1111, 715, 623, 2189, 2394, 405, 1158, 2006, 692, 1861, 418, 2558, 1123, 879, 1627, 2211, 339, 1058, 2045, 272, 2525, 2279, 1851, 62, 714, 1986, 1527, 351, 764, 256, 1182, 838, 949, 2067, 2425, 1923, 7, 636, 582, 710, 2200, 1804, 2300, 1849, 721, 720, 2496, 2427, 902, 1547, 2183, 2179, 1176, 833, 973, 617, 2550, 2254, 1420, 401, 1866, 2048, 1705, 189, 807, 1153, 1303, 454, 2000, 1594, 842, 57, 1609, 380, 606, 1868, 213, 1449, 1796, 1682, 821, 832, 1841, 1622, 1251, 2435, 1105, 86, 1532, 896, 2196, 2560, 2627, 535, 223, 1214, 1543, 253, 1542, 676, 1863, 1246, 2449, 1794, 2328, 469, 843, 740, 1034, 18, 598, 1587, 300, 2508, 1733, 158, 1857, 262, 2385, 1048, 680, 1672, 1140, 2463, 2636, 55, 805, 2024, 1101, 1961, 1755, 1005, 2018, 539, 993, 634, 112, 1955, 1759, 1423, 2434, 1537, 82, 1581, 1393, 928, 457, 137, 2106, 259, 1888, 2364, 2340, 2151, 1226, 1698, 1572, 398, 594, 860, 1042, 769, 2175, 1028, 327, 25, 551, 2583, 229, 630, 669, 1574, 2208, 1826, 2086, 874, 1965, 297, 1816, 1665, 1177, 1132, 672, 758, 1518, 1885, 2396, 2095, 2355, 922, 2507, 1812, 2415, 1096, 866, 719, 1765, 2589, 2169, 565, 2540, 934, 2510, 395, 1506, 605, 1642, 1947, 1576, 574, 1145, 1989, 2501, 995, 1332, 2145, 1935, 2184, 2420, 702, 1763, 1426, 1702, 1906, 767, 611, 109, 172, 2614, 345, 11, 1920, 901, 1401, 415, 2504, 2218, 854, 1306, 2115, 2288, 2335, 2325, 269, 662, 1163, 496, 1350, 566, 2247, 106, 386, 1149, 1919, 596, 1524, 2469, 1719, 362, 813, 559, 1709, 628, 2358, 2445, 2606, 1098, 935, 4, 2497, 2532, 2046, 1659, 1241, 1859, 886, 1983, 199, 2633, 2257, 2112, 1930, 273, 41, 1108, 2440, 1491, 1945, 1655, 1444, 320, 2442, 2466, 2378, 1441, 1893, 2084, 1606, 1458, 356, 123, 92, 1135, 2577, 1848, 2021, 1929, 1362, 95, 1683, 2522, 954, 1402, 2271, 2281, 296, 1075, 1742, 26, 2568, 435, 2293, 2049, 1569, 168, 516, 309, 718, 1282, 2491, 2476, 1616, 1490, 1207, 134, 2193, 1944, 1396, 1938, 2400, 597, 23, 138, 1010, 113, 1803, 1015, 399, 1468, 1043, 1704, 2015, 183, 187, 279, 2363, 915, 1948, 35, 1481, 330, 988, 1907, 1500, 2499, 1200, 837, 1256, 1545, 2209, 964, 665, 603, 1881, 1899, 2414, 378, 178, 1294, 2201, 562, 540, 1055, 1471, 2206, 232, 2398, 375, 364, 1326, 938, 586, 2626, 471, 504, 2226, 1972, 147, 2356, 992, 218, 1085, 1276, 1539, 276, 642, 909, 1700, 1901, 2519, 751, 534, 2083, 1772, 2272, 266, 847, 2490, 1089, 790, 1229, 1474, 403, 875, 1519, 1477, 661, 2357, 2019, 791, 460, 546, 2404, 154, 681, 1437, 1021, 2005, 2381, 1827, 1966, 1751, 1127, 1219, 1730, 2372, 2098, 1156, 1266, 2578, 69, 1218, 1994, 1080, 639, 2359, 2331, 1909, 839, 2517, 234, 743, 1056, 1595, 283, 343, 1726, 40, 869, 781, 2122, 608, 425, 1582, 1760, 509, 1715, 1368, 75, 165, 1002, 1388, 1512, 1874, 1890, 2059, 2339, 448, 157, 1567, 905, 942, 2004, 2321, 12, 712, 1073, 560, 2307, 542, 2615, 2590, 690, 2529, 505, 2082, 1744, 1631, 315, 487, 1196, 1652, 1555, 2368, 196, 416, 2486, 34, 74, 1381, 295, 368, 970, 144, 308, 2464, 1992, 118, 1610, 1822, 1669, 5, 2198, 2556, 1786, 2221, 1242, 1407, 2609, 828, 779, 2085, 900, 1921, 243, 2234, 768, 1174, 1195, 651, 971, 2294, 1372, 1754, 840, 1531, 741, 1504, 280, 1882, 1640, 2093, 1998, 2424, 933, 1022, 1409, 650, 32, 202, 1941, 1047, 317, 2194, 132, 604, 852, 452, 2187, 2477, 2041, 819, 349, 2242, 2256, 1959, 932, 1538, 1678, 1212, 2535, 1810, 410, 1549, 1766, 119, 736, 2346, 1918, 396, 1892, 372, 738, 98, 2548, 2392, 1612, 894, 1429, 622, 1016, 2637, 270, 1820, 1522, 302, 2239, 2161, 442, 1397, 337, 637, 251, 1217, 1757, 1696, 2268, 731, 1121, 1695, 1536, 1384, 38, 1953, 1258, 1886, 2478, 477, 2387, 1206, 762, 1637, 338, 2276, 1830, 1320, 1151, 632, 1172, 139, 2036, 43, 2075, 374, 340, 2631, 1615, 2233, 1691, 910, 1008, 252, 1568, 1814, 2243, 1690, 2453, 2575, 756, 872, 2506, 1216, 1033, 2329, 2138, 325, 180, 1094, 856, 2602, 1779, 699, 494, 614, 563, 2119, 1783, 1792, 1910, 1130, 1990, 318, 2052, 2278, 408, 864, 1299, 1451, 222, 1167, 1680, 785, 1190, 1317, 2091, 688, 1971, 957, 2302, 1600, 2296, 24, 2538, 444, 1530, 814, 1463, 1526, 64, 946, 1579, 1488, 2301, 2343, 1979, 2360, 484, 2101, 430, 2351, 2530, 260, 2252, 613, 1161, 170, 958, 1657, 2181, 385, 581, 750, 2011, 2113, 1442, 777, 693, 1147, 2332, 2458, 2171, 510, 2450, 411, 557, 834, 1707, 479, 346, 2192, 197, 2078, 254, 2009, 289, 124, 1088, 360, 1239, 2438, 2557, 1513, 2409, 884, 1026, 722, 130, 1459, 2312, 1727, 1386, 1789, 2584, 893, 1178, 1071, 247, 1809, 519, 93, 627, 1109, 142, 1653, 2066, 1731, 1124, 1604, 709, 1807, 711, 393, 390, 2405, 27, 1607, 759, 2353, 590, 1164, 1740, 2253, 1936, 2071, 2566, 1981, 1418, 1181, 775, 802, 1380, 2370, 1873, 631, 1263, 811, 1507, 136, 2412, 1852, 1013, 2493, 379, 677, 76, 1006, 1833, 1436, 2624, 1846, 2197, 663, 514, 527, 2467, 481, 249, 982, 2417, 1865, 956, 1693, 1469, 49, 1255, 2338, 2563, 2305, 1962, 1752, 1801, 2410, 739, 538, 1586, 258, 467, 1720, 2153, 1515, 348, 73, 978, 1197, 835, 2270, 1053, 550, 1922, 2320, 2146, 1375, 2003, 2333, 2462, 61, 629, 2102, 1525, 1019, 907, 1434, 2581, 1954, 1427, 2299, 2124, 1758, 2369, 1334, 734, 2344, 1891, 1278, 265, 1580, 532, 1514, 1031, 1024, 381, 458, 1309, 770, 215, 2402, 873, 2521, 1413, 1160, 980, 216, 2456, 1819, 1456, 929, 2317, 285, 1825, 836, 314, 257, 1146, 1017, 1405, 544, 1904, 679, 1045, 2330, 1184, 950, 2515, 1382, 453, 1062, 1009, 655, 1155, 175, 2, 2265, 2167, 2594, 1785, 1916, 587, 571, 1095, 1711, 830, 493, 2219, 1974, 867, 1831, 2619, 1403, 282, 111, 2165, 638, 626, 766, 321, 423, 1588, 80, 573, 2074, 1227, 2423, 1475, 742, 2060, 2527, 2613, 2114, 1564, 2121, 2595, 1741, 84, 558, 414, 1790, 1107, 2246, 1470, 10, 1162, 2554, 294, 2588, 1664, 2354, 961, 1297, 2366, 1391, 2612, 1897, 492, 366, 503, 332, 1608, 760, 2280, 2334, 2274, 1250, 164, 2413, 2617, 1099, 169, 2026, 1296, 706, 207, 1240, 2292, 219, 3, 666, 316, 885, 2180, 1483, 2057, 1933, 1152, 1718, 1784, 953, 2465, 567, 131, 1193, 803, 1464, 1750, 1832, 1398, 890, 2545, 2439, 498, 774, 145, 522, 792, 122, 1324, 45, 2432, 2016, 1880, 1067, 500, 66, 1287, 2433, 1559, 255, 30, 359, 1128, 515, 1134, 1288, 328, 244, 156, 1387, 2054, 1533, 2096, 1395, 450],
    _phantom: core::marker::PhantomData,
};

pub(super) static NAME_TO_IDX: PhfMap<[u8], u16> = PhfMap {
    key: 10656210946825422025,
    disps: &[(0, 15), (0, 0), (0, 0), (0, 16), (0, 1), (0, 1), (0, 4), (0, 0), (0, 7), (0, 0), (0, 1), (0, 0), (0, 7), (0, 4), (0, 0), (0, 41), (0, 0), (0, 54), (0, 4), (0, 99), (0, 6), (0, 18), (0, 14), (0, 10), (0, 2), (0, 0), (0, 3), (0, 42), (0, 0), (0, 7), (0, 0), (0, 47), (0, 3), (0, 14), (0, 83), (0, 3), (0, 60), (0, 65), (0, 2), (0, 2), (0, 4), (0, 4), (0, 2), (0, 1), (0, 10), (0, 0), (0, 3), (0, 56), (0, 0), (0, 4), (0, 0), (0, 2), (0, 0), (0, 0), (0, 0), (0, 5), (0, 11), (0, 0), (0, 4), (0, 32), (0, 3), (0, 2), (0, 1), (0, 0), (0, 5), (0, 9), (0, 0), (0, 4), (0, 2), (0, 1), (0, 2), (0, 39), (0, 0), (0, 3), (0, 6), (0, 0), (0, 9), (0, 6), (0, 0), (0, 19), (0, 23), (0, 2), (0, 15), (0, 11), (0, 16), (0, 0), (0, 30), (0, 3), (0, 0), (0, 0), (0, 57), (0, 2), (0, 9), (0, 11), (0, 1), (0, 1), (0, 2), (0, 26), (0, 2), (0, 3), (0, 13), (0, 3), (0, 0), (0, 26), (0, 0), (0, 0), (0, 0), (0, 0), (0, 1), (0, 4), (0, 10), (0, 12), (0, 105), (0, 3), (0, 2), (0, 7), (0, 1), (0, 0), (0, 2), (0, 0), (0, 15), (0, 25), (0, 0), (0, 0), (0, 5), (0, 19), (0, 0), (0, 33), (0, 9), (0, 5), (0, 4), (0, 7), (0, 8), (0, 3), (0, 0), (0, 36), (0, 1), (0, 2), (0, 10), (0, 65), (0, 12), (0, 18), (0, 1), (0, 5), (0, 28), (0, 0), (0, 0), (0, 70), (0, 18), (0, 29), (0, 35), (0, 2), (0, 0), (0, 16), (0, 2), (0, 0), (0, 36), (0, 1), (0, 10), (0, 4), (0, 26), (0, 3), (0, 6), (0, 4), (0, 31), (0, 1), (0, 64), (0, 345), (0, 65), (0, 1), (0, 1), (0, 65), (0, 0), (0, 14), (0, 22), (0, 6), (0, 12), (0, 8), (0, 0), (0, 46), (0, 43), (0, 0), (0, 80), (0, 55), (0, 530), (0, 1), (0, 14), (0, 12), (0, 5), (0, 5), (0, 0), (0, 39), (0, 3), (0, 10), (0, 15), (0, 18), (0, 6), (0, 0), (0, 6), (0, 0), (0, 18), (0, 0), (0, 3), (0, 14), (0, 50), (0, 0), (0, 8), (0, 36), (0, 0), (0, 2), (0, 0), (0, 60), (0, 28), (0, 30), (0, 5), (0, 23), (0, 11), (0, 0), (0, 11), (0, 5), (0, 3), (0, 30), (0, 423), (0, 41), (0, 0), (0, 146), (0, 2), (0, 81), (0, 3), (0, 1), (0, 0), (0, 9), (0, 13), (0, 27), (0, 0), (0, 3), (0, 118), (0, 17), (0, 22), (0, 2), (0, 2), (0, 99), (0, 2), (0, 102), (0, 18), (0, 0), (0, 19), (0, 37), (0, 0), (0, 11), (0, 127), (0, 1), (0, 7), (0, 36), (0, 14), (0, 29), (0, 2), (0, 79), (0, 5), (0, 0), (0, 0), (0, 23), (0, 9), (0, 0), (0, 0), (0, 6), (0, 0), (0, 50), (0, 0), (0, 0), (0, 23), (0, 0), (0, 7), (0, 1), (0, 1), (0, 80), (0, 8), (0, 19), (0, 12), (0, 1), (0, 1), (0, 6), (0, 0), (0, 10), (0, 2), (0, 45), (0, 2), (0, 21), (0, 1), (0, 2), (0, 1), (0, 9), (0, 33), (0, 0), (0, 8), (0, 48), (0, 74), (0, 0), (0, 0), (0, 282), (0, 301), (0, 0), (0, 0), (0, 5), (0, 4), (0, 52), (0, 23), (0, 0), (0, 19), (0, 8), (0, 7), (0, 7), (0, 8), (0, 5), (0, 0), (0, 0), (0, 0), (0, 415), (0, 0), (0, 19), (0, 0), (0, 51), (0, 2), (0, 2), (0, 13), (0, 1), (0, 86), (0, 0), (0, 14), (0, 19), (0, 52), (0, 0), (0, 4), (0, 13), (0, 4), (0, 92), (0, 0), (0, 14), (0, 13), (0, 0), (0, 45), (0, 0), (0, 19), (0, 14), (0, 5), (0, 5), (0, 1), (0, 1), (0, 36), (0, 15), (0, 0), (0, 72), (0, 2), (0, 7), (0, 21), (0, 26), (0, 0), (0, 4), (0, 0), (0, 20), (0, 6), (0, 31), (0, 0), (0, 0), (0, 0), (0, 40), (0, 68), (0, 0), (0, 0), (0, 2), (0, 0), (0, 42), (0, 1), (0, 4), (0, 4), (0, 0), (0, 3), (0, 0), (0, 86), (0, 175), (0, 156), (0, 15), (0, 9), (0, 293), (0, 40), (0, 627), (0, 0), (0, 14), (0, 2), (0, 0), (0, 2), (0, 0), (0, 17), (0, 0), (0, 4), (0, 8), (0, 3), (0, 10), (0, 22), (0, 0), (0, 14), (0, 12), (0, 6), (0, 4), (0, 15), (0, 0), (0, 38), (0, 25), (0, 11), (0, 12), (0, 1), (0, 2), (0, 178), (0, 5), (0, 9), (0, 7), (0, 2), (0, 124), (0, 339), (0, 3), (0, 23), (0, 149), (0, 0), (0, 3), (0, 4), (0, 0), (0, 6), (0, 0), (0, 30), (0, 31), (0, 0), (0, 1), (0, 19), (0, 0), (0, 1), (0, 563), (0, 7), (0, 0), (0, 4), (0, 110), (0, 14), (0, 2), (0, 30), (0, 2), (0, 7), (0, 0), (0, 0), (0, 14), (0, 7), (0, 16), (0, 57), (0, 8), (0, 6), (0, 4), (0, 2), (0, 121), (0, 0), (0, 2), (0, 454), (0, 0), (0, 4), (0, 392), (0, 7), (0, 34), (0, 240), (0, 4), (0, 2), (0, 22), (0, 9), (0, 620), (0, 0), (0, 40), (0, 431), (0, 271), (0, 32), (0, 28), (0, 11), (0, 14), (0, 5), (0, 28), (0, 70), (0, 126), (0, 73), (0, 38), (0, 52), (0, 6), (0, 1), (0, 133), (0, 128), (0, 27), (0, 18), (0, 0), (0, 31), (0, 1), (0, 38), (0, 23), (0, 72), (0, 3), (0, 89), (0, 26), (0, 11), (0, 137), (0, 1), (0, 10), (0, 0), (0, 0), (0, 2), (0, 5), (0, 8), (0, 1), (0, 32), (0, 2), (0, 0), (0, 51), (0, 46), (0, 0), (0, 3), (0, 0), (0, 5), (0, 62), (0, 26), (0, 6), (0, 25), (0, 0), (0, 0), (0, 42), (0, 19), (0, 16), (0, 3), (0, 0), (0, 5), (0, 4), (0, 47), (0, 0), (0, 0), (0, 2), (0, 7), (0, 37), (0, 4), (0, 0), (0, 4), (0, 7), (0, 56), (0, 4), (0, 0), (0, 220), (0, 23), (0, 1), (0, 70), (0, 0), (0, 2), (0, 5), (0, 0), (0, 120), (0, 7), (0, 0), (0, 0), (0, 154), (0, 1), (0, 2), (0, 0), (0, 0), (0, 119), (0, 334), (0, 3), (0, 0), (0, 5), (0, 16), (0, 16), (0, 0), (0, 1), (0, 0), (0, 2), (0, 147), (0, 7), (0, 0), (0, 77), (0, 16), (0, 13), (0, 144), (0, 2), (0, 37), (0, 33), (0, 1), (0, 125), (0, 12), (0, 55), (0, 93), (0, 77), (0, 47), (0, 0), (0, 2), (0, 727), (0, 330), (0, 22), (0, 0), (0, 33), (0, 95), (0, 21), (0, 20), (0, 43), (0, 336), (0, 53), (0, 0), (0, 10), (0, 2), (0, 264), (0, 529), (0, 0), (0, 43), (0, 1), (0, 30), (0, 180), (0, 6), (0, 155), (0, 58), (0, 20), (0, 109), (0, 2), (0, 697), (0, 5), (0, 8), (0, 1), (0, 0), (0, 40), (0, 49), (0, 57), (0, 59), (0, 1), (0, 0), (0, 5), (0, 5), (0, 0), (0, 0), (0, 212), (0, 103), (0, 5), (0, 5), (0, 120), (0, 6), (0, 2), (0, 18), (0, 7), (0, 112), (0, 18), (0, 0), (0, 135), (0, 12), (0, 5), (0, 21), (0, 1), (0, 820), (0, 0), (0, 19), (0, 7), (0, 51), (0, 354), (0, 0), (0, 139), (0, 216), (0, 566), (0, 37), (0, 0), (0, 40), (0, 48), (0, 536), (0, 64), (0, 1), (0, 222), (0, 1), (0, 182), (0, 1), (0, 10), (0, 28), (0, 14), (0, 0), (0, 93), (0, 0), (0, 626), (0, 1000), (0, 469), (0, 1), (0, 111), (0, 977), (0, 6), (0, 0), (0, 87), (0, 218), (0, 0), (0, 3), (0, 29), (0, 32), (0, 78), (0, 0), (0, 0), (0, 51), (0, 83), (0, 0), (0, 17), (0, 6), (0, 2), (0, 2), (0, 8), (0, 39), (0, 259), (0, 3), (0, 841), (0, 43), (0, 0), (0, 25), (0, 167), (0, 7), (0, 160), (0, 1), (0, 15), (0, 41), (0, 31), (0, 4), (0, 14), (0, 208), (0, 7), (0, 4), (0, 259), (0, 216), (0, 24), (0, 0), (0, 1), (0, 1), (0, 31), (0, 14), (0, 4), (0, 0), (0, 13), (0, 2), (0, 0), (0, 1), (0, 18), (0, 0), (0, 2), (0, 530), (0, 3), (0, 48), (0, 88), (0, 13), (0, 0), (0, 192), (0, 0), (0, 102), (0, 2), (0, 1), (0, 33), (0, 6), (0, 634), (0, 100), (0, 0), (0, 9), (0, 217), (0, 0), (0, 14), (0, 894), (0, 229), (0, 3), (0, 666), (0, 160), (0, 0), (0, 0), (0, 116), (0, 947), (0, 0), (0, 12), (0, 12), (0, 12), (0, 5), (0, 3), (0, 22), (0, 2), (0, 8), (0, 0), (0, 19), (0, 24), (0, 105), (0, 230), (0, 32), (0, 713), (0, 688), (0, 0), (0, 600), (0, 6), (0, 26), (0, 12), (0, 12), (0, 11), (0, 13), (0, 8), (0, 112), (0, 13), (0, 25), (0, 0), (0, 197), (0, 0), (0, 39), (0, 81), (0, 7), (0, 1), (0, 6), (0, 0), (0, 10), (0, 285), (0, 42), (0, 20), (0, 217), (0, 732), (0, 16), (0, 1031), (0, 26), (0, 54), (0, 29), (0, 19), (0, 0), (0, 3), (0, 3), (0, 2), (0, 212), (0, 3), (0, 980), (0, 0), (0, 3), (0, 593), (0, 7), (0, 0), (0, 150), (0, 105), (0, 150), (0, 967), (0, 1027), (0, 154), (0, 7), (0, 3), (0, 454), (0, 211), (0, 4), (0, 12), (0, 1160), (0, 0), (0, 128), (0, 42), (0, 131), (0, 139), (0, 48), (0, 70), (0, 94), (0, 422), (0, 775), (0, 21), (0, 95), (0, 0), (0, 497), (0, 216), (0, 808), (0, 0), (0, 2), (0, 3), (0, 301), (0, 2334), (0, 4), (0, 335), (0, 1), (0, 40), (0, 61), (0, 10), (0, 0), (0, 3), (0, 3), (0, 57), (0, 267), (0, 0), (0, 1), (0, 484), (0, 2221), (0, 0), (0, 1), (0, 38), (0, 2), (0, 1), (0, 2309), (0, 8), (0, 4), (0, 23), (0, 1), (0, 0), (0, 7), (0, 130), (0, 411), (0, 6), (0, 248), (0, 849), (0, 1273), (0, 56), (0, 1626), (0, 11), (0, 0), (0, 45), (0, 560), (0, 1968), (0, 2493), (0, 846), (0, 347), (0, 40), (0, 0), (0, 362), (0, 19), (0, 4), (0, 104), (0, 428), (0, 26), (0, 165), (0, 439), (0, 13), (0, 2229), (0, 0), (0, 21), (0, 1728), (0, 19), (0, 2343), (0, 132), (0, 456), (0, 8), (0, 22), (0, 628), (0, 174), (0, 40), (0, 89), (0, 145), (0, 11), (0, 11), (0, 0), (0, 0), (0, 94), (0, 0), (0, 0), (0, 14), (0, 1644), (0, 899), (0, 46), (0, 204), (0, 8), (0, 0), (0, 0), (0, 1333), (0, 7), (0, 1745), (0, 0), (0, 380), (0, 0), (0, 4), (0, 673), (0, 72), (0, 9), (0, 1319), (0, 0), (0, 37), (0, 126), (0, 0), (0, 1), (0, 120), (0, 0), (0, 4), (0, 25), (0, 35), (0, 38), (0, 1), (0, 1), (0, 44), (0, 3), (0, 149), (0, 5), (0, 202), (0, 43), (0, 36), (0, 1957), (0, 647), (0, 0), (0, 1253), (0, 3), (0, 20), (0, 64), (0, 69), (0, 27), (0, 8), (0, 35), (0, 883), (0, 10), (0, 52), (0, 4), (0, 172), (0, 3), (0, 2), (0, 2), (0, 0), (0, 19), (0, 393), (0, 5), (0, 2), (0, 193), (0, 1), (0, 9), (0, 12), (0, 4), (0, 107), (0, 0), (0, 25), (0, 27), (0, 186), (0, 76), (0, 1317), (0, 336), (0, 74), (0, 355), (0, 676), (0, 1862), (0, 1020), (0, 82), (0, 39), (0, 38)],
    map: &[529, 2133, 1191, 420, 255, 654, 2074, 807, 618, 774, 182, 1425, 2420, 171, 1832, 660, 2089, 36, 2267, 1935, 370, 1169, 2500, 563, 890, 1303, 1751, 1574, 132, 1000, 692, 1350, 1658, 1667, 1716, 262, 2146, 2616, 1929, 1452, 1153, 729, 167, 349, 1533, 1332, 557, 2624, 131, 355, 1243, 475, 51, 2439, 1736, 385, 2475, 1370, 502, 282, 1732, 2551, 579, 473, 1376, 1188, 96, 665, 2001, 681, 2544, 2402, 596, 2287, 1649, 451, 960, 2471, 1958, 1294, 1570, 2491, 2518, 2554, 1061, 130, 1408, 79, 1333, 1798, 1564, 2210, 1569, 2409, 2282, 1680, 1409, 917, 98, 545, 1541, 1468, 367, 1321, 1580, 1500, 1867, 2313, 2066, 2547, 1893, 1760, 966, 1804, 333, 1724, 1609, 1365, 2466, 2113, 1357, 2557, 721, 1755, 1097, 1542, 2106, 921, 339, 2534, 316, 2479, 2374, 1017, 1290, 2097, 1780, 2459, 254, 1546, 1484, 2196, 346, 1393, 2555, 2005, 739, 2117, 2450, 1841, 819, 2329, 2412, 2105, 687, 2583, 2598, 1448, 2426, 26, 2224, 2574, 1464, 89, 2318, 797, 1765, 1843, 861, 2273, 940, 427, 1059, 2550, 2243, 1994, 1834, 436, 293, 2317, 1075, 605, 1502, 2499, 29, 2138, 207, 2505, 583, 1727, 2228, 1339, 1688, 25, 2441, 1020, 43, 922, 337, 2256, 2361, 2460, 598, 156, 395, 2501, 2068, 2516, 2058, 1623, 994, 1931, 415, 275, 2132, 2004, 406, 2445, 871, 1278, 2463, 1419, 733, 1495, 1305, 648, 2354, 1337, 426, 1768, 2102, 38, 695, 2634, 2219, 1720, 1887, 2298, 205, 2407, 1792, 1507, 390, 878, 995, 300, 460, 1015, 924, 2073, 2055, 230, 2399, 248, 2362, 1167, 621, 977, 986, 471, 1282, 2172, 364, 1257, 1161, 1629, 53, 161, 261, 1526, 1106, 412, 712, 1905, 962, 2101, 322, 1498, 175, 971, 2521, 2029, 1113, 2170, 1605, 151, 531, 1066, 2151, 844, 1763, 214, 139, 1517, 2617, 1519, 745, 1447, 887, 449, 2347, 1709, 640, 292, 2213, 2588, 81, 2627, 2366, 1417, 129, 1742, 954, 1353, 1380, 857, 1806, 2125, 2568, 485, 1690, 2120, 1396, 872, 1941, 154, 783, 399, 1288, 674, 28, 565, 805, 495, 222, 440, 829, 2174, 2082, 1039, 1286, 914, 1741, 2391, 659, 403, 1949, 134, 187, 2052, 45, 1462, 2166, 680, 603, 1264, 137, 1, 469, 1254, 2489, 1620, 2307, 1067, 2581, 1429, 2424, 772, 2567, 2173, 217, 2116, 2388, 1481, 1923, 810, 1613, 2160, 302, 2549, 1920, 613, 1157, 1092, 1769, 2109, 1748, 1024, 898, 1444, 467, 1921, 1677, 1666, 1022, 1131, 1892, 2300, 1651, 957, 1738, 763, 500, 506, 820, 1885, 778, 2114, 2350, 556, 667, 2290, 2137, 974, 862, 1132, 956, 6, 2345, 351, 2183, 1359, 2249, 162, 1340, 688, 1646, 513, 1485, 860, 690, 1098, 2028, 1611, 2169, 885, 1038, 1997, 1307, 127, 2386, 1784, 1103, 1524, 509, 40, 1753, 332, 1142, 2486, 108, 2293, 991, 158, 756, 99, 2523, 1823, 1661, 867, 1797, 2047, 543, 628, 1571, 1954, 393, 1776, 816, 1656, 195, 514, 1215, 1707, 1342, 852, 814, 1208, 1280, 1461, 1535, 1052, 2083, 1782, 909, 350, 1478, 629, 2435, 1979, 1913, 1637, 1551, 258, 595, 678, 1995, 1881, 1027, 1991, 1785, 2280, 2363, 755, 845, 2134, 620, 537, 201, 1120, 1094, 365, 584, 1387, 948, 157, 1220, 567, 1759, 2587, 198, 1492, 2158, 1928, 1301, 724, 1354, 1349, 55, 1326, 1835, 1265, 1683, 2383, 2222, 569, 1734, 1391, 1833, 2153, 2037, 901, 1701, 1228, 1728, 368, 2582, 359, 576, 709, 1238, 1053, 1946, 121, 581, 627, 1137, 14, 927, 95, 1674, 388, 1042, 1182, 47, 329, 174, 561, 1151, 2480, 1912, 1174, 2197, 1869, 730, 2305, 1673, 2601, 2442, 2371, 883, 1597, 2357, 841, 422, 120, 2506, 952, 808, 700, 2311, 2625, 2536, 2214, 2309, 1003, 720, 2593, 2043, 2078, 2064, 1005, 1689, 2145, 2046, 535, 850, 786, 711, 1940, 1230, 2328, 378, 1641, 751, 1726, 2390, 3, 1573, 1664, 809, 5, 935, 2302, 1148, 1116, 2488, 1160, 548, 1081, 2517, 1685, 2233, 1722, 2258, 246, 1774, 1583, 1933, 429, 1196, 540, 1296, 348, 2342, 481, 1112, 361, 2496, 1445, 961, 570, 439, 2563, 1048, 559, 992, 1040, 1523, 1063, 1171, 2527, 492, 1996, 1071, 1458, 2333, 58, 2538, 1978, 204, 2389, 591, 1465, 102, 2025, 31, 1204, 1346, 1586, 1250, 1245, 1459, 2096, 2033, 2206, 718, 1866, 356, 858, 1762, 1699, 2493, 2111, 2623, 2437, 875, 57, 1626, 110, 2198, 1309, 2156, 1807, 1548, 1553, 1540, 2276, 633, 4, 916, 266, 1272, 1252, 2107, 2369, 748, 1872, 397, 185, 2419, 1126, 1091, 1983, 1691, 708, 1771, 2176, 2060, 780, 69, 443, 1849, 652, 1632, 947, 1302, 1315, 483, 2349, 446, 2414, 1323, 243, 2221, 1520, 1109, 1072, 1183, 1345, 288, 1410, 1119, 1950, 1172, 1910, 232, 2194, 1829, 2212, 2189, 968, 2340, 1655, 49, 1135, 526, 2621, 1700, 638, 236, 760, 1813, 84, 336, 1037, 768, 159, 2220, 900, 505, 2448, 2010, 1986, 2192, 1176, 2093, 497, 496, 1781, 792, 2094, 283, 1073, 2511, 1640, 1438, 1382, 2119, 1360, 503, 716, 944, 1145, 1621, 1825, 2428, 2150, 2564, 226, 1421, 790, 2572, 1263, 752, 1014, 753, 1118, 761, 1479, 2562, 343, 1890, 1402, 769, 1456, 726, 918, 34, 1547, 2296, 311, 2467, 2355, 2279, 1451, 943, 2304, 1029, 796, 1982, 2205, 1442, 328, 510, 317, 1205, 1968, 897, 666, 1206, 1585, 1102, 2131, 518, 113, 2203, 1916, 425, 880, 1508, 744, 604, 1090, 1400, 1270, 1311, 953, 1821, 1714, 1638, 231, 1855, 60, 1868, 2596, 1436, 247, 2331, 1703, 936, 2242, 318, 2091, 1558, 1432, 234, 1034, 2050, 623, 1604, 1503, 2529, 1545, 2591, 1070, 1026, 269, 67, 1622, 1902, 1083, 1670, 1446, 2248, 2338, 1466, 1715, 564, 1531, 1883, 2080, 873, 2325, 76, 1506, 438, 707, 326, 2187, 2181, 767, 912, 1555, 289, 1567, 779, 1594, 1884, 1218, 1671, 574, 148, 1631, 2286, 863, 1970, 913, 1197, 658, 1487, 848, 2600, 2178, 1969, 2070, 1371, 71, 899, 106, 1334, 2580, 334, 1076, 256, 1645, 1665, 946, 2000, 915, 1266, 2560, 1864, 2408, 1390, 1708, 138, 206, 430, 2208, 62, 371, 358, 2180, 2186, 1232, 706, 2540, 86, 2353, 35, 308, 1202, 1659, 416, 421, 1616, 2507, 1733, 1159, 2606, 2619, 2081, 2195, 1100, 383, 2218, 101, 534, 104, 2062, 1249, 218, 1862, 1624, 717, 30, 1775, 2099, 215, 1486, 544, 622, 1489, 441, 1799, 170, 1273, 1744, 2401, 859, 2016, 1737, 2136, 2326, 18, 533, 1966, 2281, 2193, 764, 866, 2009, 2121, 1236, 1412, 1710, 2288, 1198, 463, 1049, 2443, 2546, 1687, 1873, 2358, 1787, 2268, 520, 2246, 345, 1130, 1420, 2100, 886, 486, 122, 2433, 1529, 1009, 1262, 2520, 1471, 1577, 133, 1164, 1579, 601, 523, 1364, 1698, 354, 1214, 1904, 1457, 1300, 270, 1878, 1016, 1630, 136, 1068, 1568, 1521, 405, 704, 1974, 1054, 1915, 868, 1104, 2519, 413, 2274, 225, 1241, 590, 2454, 417, 487, 617, 2124, 2495, 697, 2570, 2122, 1253, 945, 955, 387, 376, 1443, 1697, 1777, 1166, 931, 437, 2086, 10, 972, 1460, 1618, 78, 787, 580, 734, 2177, 2014, 2323, 2376, 114, 577, 1441, 831, 2503, 2126, 1930, 2141, 1907, 1433, 1828, 2021, 713, 981, 1095, 684, 1259, 1932, 2314, 1617, 310, 1152, 394, 1367, 833, 1758, 2241, 1474, 73, 644, 1684, 1838, 424, 2337, 219, 1330, 276, 1319, 1472, 1074, 631, 1140, 2525, 1704, 882, 1285, 141, 1431, 2041, 2602, 1557, 920, 431, 280, 1388, 409, 784, 2423, 568, 714, 1185, 2482, 88, 1224, 770, 1532, 1603, 842, 338, 694, 586, 274, 1352, 1162, 766, 37, 223, 832, 165, 1271, 1374, 2487, 941, 1686, 296, 2275, 1180, 2270, 822, 2626, 2238, 1601, 434, 932, 281, 825, 1922, 20, 2440, 2027, 828, 2531, 1258, 400, 562, 2188, 1717, 1289, 147, 2034, 686, 1415, 2630, 1514, 1101, 146, 762, 1277, 1199, 727, 597, 578, 803, 2031, 987, 1411, 1706, 1062, 1746, 2452, 619, 401, 116, 249, 461, 1013, 2622, 910, 2631, 1811, 635, 172, 588, 2148, 2461, 656, 1368, 2492, 942, 227, 781, 1581, 1060, 183, 319, 1193, 340, 1880, 525, 538, 2237, 1239, 2635, 1195, 837, 2545, 2613, 1210, 2139, 1327, 2289, 740, 233, 2456, 257, 2370, 242, 1796, 2211, 625, 259, 1477, 1840, 1528, 1925, 408, 1794, 2586, 2533, 2087, 2413, 2312, 1256, 2510, 587, 2315, 2398, 250, 1743, 1304, 352, 2175, 384, 1924, 1773, 2403, 80, 2022, 2003, 2321, 1971, 200, 1125, 2406, 239, 1379, 1209, 1322, 653, 2244, 2227, 1213, 610, 1397, 2185, 2259, 2605, 1859, 123, 111, 702, 1386, 1414, 1990, 1967, 1287, 479, 1381, 2597, 840, 2462, 267, 1672, 1117, 1602, 1536, 1973, 188, 846, 2393, 260, 2140, 1030, 22, 951, 679, 124, 290, 2263, 1363, 173, 2039, 1607, 1394, 553, 1123, 221, 926, 1980, 2343, 1951, 2421, 647, 651, 1914, 1844, 1572, 103, 39, 1274, 1036, 458, 2223, 94, 1261, 2075, 1084, 1959, 1903, 1595, 1560, 1356, 1808, 447, 2368, 244, 1625, 404, 2254, 908, 1168, 418, 1231, 1633, 1491, 119, 2127, 1007, 757, 2, 2476, 990, 2336, 186, 50, 1975, 65, 184, 144, 1861, 312, 1837, 1375, 1454, 2469, 1851, 794, 2157, 8, 478, 2226, 650, 2571, 1963, 273, 925, 1926, 1518, 135, 1192, 291, 494, 1078, 2608, 2483, 2438, 82, 398, 177, 1134, 1056, 615, 1306, 301, 325, 902, 1251, 765, 982, 799, 2514, 865, 798, 743, 1619, 1588, 2231, 1044, 2316, 1977, 2629, 1917, 611, 2509, 847, 1530, 228, 2278, 373, 1317, 939, 2411, 2294, 2207, 547, 2535, 2636, 1276, 536, 1331, 671, 2458, 457, 530, 1865, 2410, 1752, 775, 1989, 2019, 90, 1593, 984, 2079, 1981, 118, 2251, 1961, 1173, 466, 379, 516, 573, 2446, 2607, 864, 1527, 1246, 1783, 1955, 800, 539, 1944, 128, 2319, 2561, 838, 1516, 973, 1681, 2069, 1355, 2295, 1544, 2592, 2618, 793, 2072, 1599, 637, 1972, 2515, 2397, 2637, 737, 442, 1407, 16, 456, 229, 701, 1676, 1377, 1138, 164, 2464, 2609, 93, 211, 1341, 2404, 1255, 382, 490, 24, 1455, 830, 1725, 1669, 851, 2297, 2394, 1652, 1767, 107, 877, 804, 1006, 1361, 907, 2603, 950, 1764, 2508, 17, 2427, 42, 2451, 1057, 693, 41, 2110, 1679, 2348, 253, 1818, 335, 1576, 1372, 978, 1463, 1011, 1344, 1754, 2255, 2152, 2236, 1055, 2103, 616, 2579, 1606, 2202, 2365, 2299, 315, 197, 795, 199, 1496, 2266, 2088, 2604, 2504, 1879, 614, 66, 1953, 1201, 52, 1534, 1589, 2497, 1561, 1147, 1802, 2059, 2032, 1591, 2595, 1085, 2589, 1227, 362, 1803, 2065, 782, 1385, 1801, 2182, 488, 474, 1786, 1399, 2381, 237, 2030, 1719, 2341, 1308, 1124, 1129, 68, 252, 1335, 983, 414, 824, 624, 791, 1723, 930, 2159, 2474, 320, 1050, 2285, 2417, 691, 1911, 1012, 2565, 2013, 419, 560, 2044, 663, 1600, 61, 2264, 1770, 2344, 2308, 1008, 789, 1281, 464, 2115, 189, 100, 528, 2162, 1233, 2481, 1692, 150, 307, 1269, 997, 2067, 1550, 109, 2620, 1234, 2351, 1449, 452, 937, 2036, 342, 2291, 2163, 675, 728, 1175, 1146, 265, 593, 1937, 445, 2048, 2049, 216, 2229, 331, 484, 1896, 1816, 1428, 2356, 1211, 271, 1660, 1240, 203, 2271, 558, 1351, 1745, 2377, 1812, 115, 155, 1735, 1590, 594, 2444, 1395, 1467, 827, 919, 2558, 1693, 788, 477, 2367, 1778, 2590, 1537, 630, 2239, 2008, 44, 636, 1473, 1065, 2147, 1711, 2384, 1960, 1222, 2143, 1021, 1141, 181, 2494, 2149, 759, 884, 344, 32, 2260, 1836, 2335, 1988, 645, 1155, 1678, 512, 599, 1086, 125, 985, 834, 1612, 59, 2537, 1002, 272, 450, 1490, 1952, 1965, 330, 1497, 2573, 491, 2095, 1187, 117, 1992, 1343, 2542, 321, 1750, 2425, 1284, 472, 112, 2301, 1035, 453, 33, 662, 1772, 1877, 1627, 389, 1122, 2002, 2142, 1511, 2085, 2434, 1225, 1889, 1082, 1430, 2372, 1639, 685, 54, 2465, 1165, 1004, 196, 1348, 2017, 1908, 1945, 2352, 2024, 1163, 542, 295, 856, 2400, 1874, 1310, 749, 551, 1695, 806, 163, 70, 641, 1406, 63, 589, 1046, 327, 646, 1316, 2382, 1186, 1790, 2167, 566, 1043, 1405, 870, 546, 2272, 2077, 1482, 1909, 286, 263, 928, 2252, 1413, 2524, 1587, 1207, 1018, 1194, 1235, 2061, 964, 1378, 643, 776, 190, 989, 1740, 2526, 1839, 2123, 2360, 1942, 1136, 160, 639, 869, 448, 1985, 169, 235, 1718, 2415, 2057, 1552, 1870, 2071, 571, 391, 1177, 220, 2200, 1031, 519, 1237, 2447, 1299, 304, 2552, 874, 1494, 2168, 192, 143, 1108, 2240, 1644, 1107, 1019, 1392, 668, 2084, 193, 1347, 1858, 2006, 2470, 1440, 2477, 153, 747, 2012, 377, 433, 843, 549, 1328, 1993, 91, 1805, 889, 892, 2191, 411, 2007, 572, 1831, 1675, 46, 2594, 1219, 592, 2128, 27, 149, 1025, 893, 2472, 97, 1439, 582, 602, 1712, 1653, 1964, 13, 279, 468, 1001, 1948, 1247, 2306, 1509, 341, 2092, 1845, 2513, 949, 1121, 402, 710, 1648, 2539, 813, 876, 1170, 9, 1757, 7, 499, 493, 1999, 575, 1150, 2284, 1615, 1938, 655, 2257, 2431, 1416, 626, 2484, 1897, 1815, 2161, 353, 1418, 2303, 1338, 202, 2612, 1901, 1987, 428, 455, 1793, 1336, 1501, 1654, 386, 1297, 1814, 2190, 251, 1668, 2292, 1795, 2473, 56, 241, 208, 366, 1731, 1592, 1582, 374, 126, 811, 285, 1244, 1058, 1554, 1628, 1854, 423, 305, 1312, 2418, 1079, 1292, 1088, 1824, 1087, 2346, 299, 812, 470, 2396, 1127, 1314, 2171, 72, 2230, 1850, 2453, 178, 999, 1217, 1747, 1144, 168, 517, 1647, 1713, 723, 2261, 2528, 2154, 277, 306, 933, 2129, 715, 1096, 507, 1427, 773, 1984, 83, 725, 1566, 826, 462, 1894, 245, 444, 1403, 2324, 2164, 854, 888, 2339, 191, 2416, 1863, 1810, 552, 454, 1538, 1876, 2498, 2449, 2364, 732, 360, 2063, 938, 2054, 1848, 2569, 19, 145, 970, 993, 1610, 2615, 1766, 482, 1051, 550, 742, 323, 1064, 835, 2478, 1212, 1525, 1313, 2090, 677, 1267, 2436, 1702, 1453, 2277, 521, 1779, 2265, 1248, 1856, 2378, 1539, 527, 140, 2204, 736, 1729, 642, 314, 48, 2234, 969, 689, 1041, 738, 1634, 2042, 722, 657, 1324, 2632, 1918, 1875, 1093, 2310, 1358, 1184, 750, 1614, 1028, 498, 1475, 649, 672, 664, 1099, 979, 1242, 268, 1563, 1800, 1522, 2468, 1423, 1827, 1657, 1663, 1860, 2541, 1565, 1179, 287, 1505, 1226, 238, 2429, 1105, 2144, 2543, 555, 2502, 1830, 758, 15, 1788, 284, 1362, 2269, 683, 2262, 988, 2215, 1404, 1283, 1435, 1819, 1291, 1450, 1200, 1325, 967, 75, 1809, 2225, 2130, 1077, 607, 839, 2522, 965, 959, 1556, 1857, 2056, 2359, 511, 2585, 2217, 1584, 2559, 2334, 2247, 1422, 180, 1298, 369, 836, 1934, 1957, 1373, 532, 2553, 815, 2614, 1424, 1133, 2455, 2015, 2392, 1114, 74, 2332, 324, 347, 1320, 1899, 465, 612, 818, 1936, 896, 1216, 2026, 904, 1739, 264, 1488, 1268, 1559, 2530, 1275, 508, 1510, 1089, 2076, 209, 2155, 2380, 1562, 1476, 504, 85, 2040, 1543, 2201, 777, 1943, 2250, 801, 1156, 2023, 1891, 1437, 608, 2035, 853, 1154, 1384, 297, 673, 731, 1898, 489, 1900, 2322, 1080, 278, 313, 1158, 1635, 1852, 213, 392, 1190, 1643, 905, 2018, 879, 2011, 1549, 1596, 21, 670, 2327, 2548, 1642, 476, 1128, 2405, 2485, 2118, 1223, 1789, 1293, 929, 1575, 1761, 432, 1906, 735, 980, 2395, 1260, 1389, 2430, 1682, 381, 2584, 2422, 1721, 1032, 1882, 2576, 2199, 1998, 696, 294, 2385, 1483, 2577, 975, 1115, 1398, 1426, 1366, 2375, 891, 1696, 1513, 1650, 817, 1512, 676, 911, 1189, 435, 105, 1756, 459, 1010, 2611, 669, 303, 1694, 298, 1181, 1853, 2283, 1023, 1515, 2599, 2578, 2051, 224, 1817, 2053, 699, 881, 1480, 2235, 541, 2209, 1434, 179, 410, 682, 240, 600, 2038, 1203, 1822, 634, 2020, 12, 1749, 2253, 609, 2575, 2633, 958, 1846, 1143, 1871, 1730, 1329, 2432, 2232, 396, 11, 821, 1820, 407, 746, 2216, 1178, 152, 2610, 2245, 77, 363, 2490, 194, 1149, 976, 1976, 176, 524, 585, 522, 1919, 802, 1636, 698, 2628, 1369, 741, 2045, 996, 1401, 92, 998, 1110, 1886, 1279, 1608, 2108, 1318, 2373, 372, 480, 2387, 2330, 895, 703, 501, 2532, 1578, 2135, 1069, 1847, 719, 632, 1895, 855, 754, 309, 1956, 554, 1033, 963, 1662, 0, 906, 1045, 142, 771, 2179, 785, 2566, 1939, 2165, 1469, 903, 923, 2320, 1842, 1499, 87, 606, 1791, 515, 1598, 1470, 1504, 2098, 1962, 1888, 2457, 705, 1295, 661, 2556, 357, 1947, 1826, 849, 1927, 2184, 894, 1229, 212, 380, 1047, 64, 823, 1139, 1221, 210, 1493, 2112, 23, 375, 2512, 1705, 2379, 1383, 934, 1111, 166, 2104],
    _phantom: core::marker::PhantomData,
};

pub(super) static LOWER_NAME_TO_IDX: PhfMap<[u8], u16> = PhfMap {
    key: 10656210946825422025,
    disps: &[(0, 63), (0, 7), (0, 174), (0, 120), (0, 54), (0, 27), (0, 129), (0, 3), (0, 6), (0, 5), (0, 47), (0, 267), (0, 27), (0, 121), (0, 0), (0, 41), (0, 23), (0, 177), (0, 0), (0, 549), (0, 19), (0, 82), (0, 48), (0, 74), (0, 0), (0, 28), (0, 130), (0, 0), (0, 47), (0, 120), (0, 311), (0, 135), (0, 0), (0, 653), (0, 3), (0, 0), (0, 41), (0, 76), (0, 23), (0, 2), (0, 0), (0, 102), (0, 385), (0, 279), (0, 1124), (0, 47), (0, 0), (0, 364), (0, 23), (0, 41), (0, 19), (0, 60), (0, 549), (0, 14), (0, 63), (0, 3), (0, 50), (0, 28), (0, 4), (0, 382), (0, 199), (0, 4), (0, 66), (0, 296), (0, 115), (0, 148), (0, 1), (0, 12), (0, 100), (0, 33), (0, 15), (0, 5), (0, 872), (0, 18), (0, 121), (0, 267), (0, 17), (0, 1), (0, 48), (0, 0), (0, 9), (0, 100), (0, 1207), (0, 557), (0, 0), (0, 11), (0, 4), (0, 340), (0, 0), (0, 1), (0, 368), (0, 28), (0, 168), (0, 5), (0, 23), (0, 20), (0, 44), (0, 8), (0, 800), (0, 27), (0, 0), (0, 0), (0, 0), (0, 2), (0, 271), (0, 309), (0, 5), (0, 0), (0, 28), (0, 211), (0, 1), (0, 470), (0, 28), (0, 2), (0, 89), (0, 21), (0, 14), (0, 56), (0, 1506), (0, 3), (0, 11), (0, 513), (0, 61), (0, 7), (0, 0), (0, 7), (0, 263), (0, 34), (0, 203), (0, 855), (0, 1), (0, 36), (0, 148), (0, 0), (0, 341), (0, 432), (0, 226), (0, 1755), (0, 141), (0, 103), (0, 166), (0, 22), (0, 0), (0, 79), (0, 1), (0, 1197), (0, 3), (0, 112), (0, 35), (0, 0), (0, 232), (0, 42), (0, 17), (0, 604), (0, 214), (0, 1), (0, 15), (0, 0), (0, 49), (0, 538), (0, 11), (0, 44), (0, 70), (0, 4), (0, 12), (0, 2), (0, 45), (0, 23), (0, 59), (0, 1), (0, 219), (0, 84), (0, 962), (0, 96), (0, 196), (0, 244), (0, 988), (0, 1325), (0, 15), (0, 5), (0, 145), (0, 56), (0, 52), (0, 188), (0, 798), (0, 1), (0, 2), (0, 2), (0, 0), (0, 818), (0, 1), (0, 201), (0, 6), (0, 81), (0, 415), (0, 29), (0, 1), (0, 7), (0, 60), (0, 0), (0, 659), (1, 0), (0, 4), (0, 2), (0, 18), (0, 82), (0, 0), (0, 5), (0, 0), (0, 1), (0, 0), (0, 58), (0, 496), (0, 45), (0, 90), (0, 17), (0, 133), (0, 32), (0, 10), (0, 0), (0, 109), (0, 125), (0, 30), (0, 1), (0, 1315), (0, 2), (0, 483), (0, 3), (0, 3), (0, 490), (0, 363), (0, 0), (0, 0), (0, 323), (0, 182), (0, 89), (0, 11), (0, 2), (0, 9), (0, 0), (0, 2), (0, 520), (0, 87), (0, 232), (0, 561), (0, 12), (0, 1619), (0, 13), (0, 1), (0, 27), (0, 336), (0, 10), (0, 3), (0, 2), (0, 36), (0, 607), (0, 67), (0, 214), (0, 1), (0, 28), (0, 5), (0, 158), (0, 246), (0, 0), (0, 1047), (0, 24), (0, 857), (0, 194), (0, 97), (0, 10), (0, 369), (0, 0), (0, 47), (0, 5), (0, 251), (0, 39), (0, 7), (0, 0), (0, 5), (0, 8), (0, 98), (0, 708), (0, 0), (0, 17), (0, 33), (1, 1009), (0, 543), (0, 138), (0, 136), (0, 516), (0, 38), (0, 1), (0, 874), (0, 2), (0, 308), (1, 894), (0, 3), (0, 0), (0, 107), (0, 436), (0, 1068), (0, 1116), (0, 156), (0, 11), (0, 4), (0, 28), (0, 1430), (0, 6), (0, 23), (0, 1313), (0, 36), (0, 18), (0, 12), (0, 804), (0, 1075), (0, 54), (1, 228), (0, 160), (0, 1948), (0, 2220), (0, 1), (0, 67), (0, 53), (0, 1087), (0, 3), (0, 100), (0, 1), (0, 1834), (0, 0), (1, 2067), (0, 26), (0, 508), (0, 1808), (0, 1), (0, 33), (0, 185), (0, 649), (0, 0), (0, 15), (1, 372), (0, 516), (0, 2135), (0, 16), (0, 8), (0, 464), (0, 3), (0, 4), (0, 141), (1, 844), (0, 684), (0, 0), (0, 212), (0, 8), (0, 0), (0, 18), (0, 224), (0, 3), (0, 10), (0, 1895), (2, 318), (0, 686), (0, 774), (0, 74), (1, 1297), (1, 2), (0, 623), (0, 12), (0, 26), (0, 6), (0, 40), (0, 1170), (0, 159), (0, 62), (0, 1783), (0, 17), (0, 7), (0, 13), (0, 0), (0, 0), (0, 1), (0, 18), (0, 20), (0, 492), (0, 635), (0, 18), (0, 145), (0, 1019), (0, 972), (0, 10), (1, 111), (0, 134), (0, 0), (0, 108), (0, 0), (0, 15), (0, 1366), (0, 18), (0, 268), (0, 101), (0, 195), (0, 118), (0, 18), (0, 116), (0, 6), (0, 31), (0, 32), (0, 8), (0, 0), (0, 3), (1, 1166), (0, 561), (0, 1482), (0, 0), (0, 19), (0, 19), (0, 13), (0, 81), (0, 147), (0, 21), (0, 50), (0, 66), (1, 29), (0, 1), (0, 311), (0, 191), (0, 102), (0, 63), (0, 98), (0, 1), (2, 160), (0, 3), (0, 11), (0, 12), (0, 126), (0, 966), (0, 0), (1, 1014), (0, 209), (1, 705), (0, 1125), (0, 1081), (0, 892), (9, 2125), (0, 1), (0, 3), (0, 371), (0, 580), (0, 0), (2, 1444), (1, 325), (0, 9), (0, 23), (0, 16), (7, 21), (0, 0), (0, 200), (0, 0), (0, 1133), (1, 1635), (0, 120), (0, 834), (0, 1001), (1, 491), (0, 1062), (1, 879), (0, 39), (0, 1), (3, 529), (0, 19), (2, 1302), (0, 1657), (0, 42), (0, 189), (0, 1859), (5, 1426), (1, 1588), (0, 899), (0, 1704), (0, 199), (3, 415), (0, 1651), (0, 0), (7, 383), (0, 922), (0, 120), (0, 0), (0, 257), (0, 0), (1, 1149), (0, 5), (0, 1161), (0, 1621), (0, 1), (0, 1121), (9, 104), (0, 576), (0, 197), (0, 2118), (3, 2205), (0, 471), (0, 635), (10, 283), (0, 11), (5, 2272), (0, 122), (0, 2), (0, 123), (9, 1854), (0, 732), (0, 0), (9, 1557), (0, 8)],
    map: &[627, 903, 426, 1971, 1585, 1073, 1221, 1720, 1145, 1303, 289, 840, 380, 1689, 1644, 1157, 432, 260, 1591, 1193, 1006, 189, 1220, 1062, 7, 973, 1791, 665, 1722, 950, 487, 2016, 1140, 1123, 216, 1165, 2292, 2336, 2000, 1660, 743, 11, 954, 677, 1423, 1079, 445, 1516, 2637, 667, 1099, 1647, 75, 1403, 2584, 1980, 1182, 190, 2419, 1824, 998, 682, 1514, 2037, 2254, 1341, 639, 494, 2162, 1805, 785, 1537, 2534, 953, 444, 2325, 2459, 2118, 1658, 1735, 1291, 670, 1912, 197, 2525, 2438, 413, 2007, 452, 693, 1259, 1121, 126, 1023, 369, 1527, 2106, 1864, 1795, 98, 888, 1498, 2313, 334, 1529, 2371, 644, 762, 250, 1311, 2161, 1296, 2296, 801, 2126, 1301, 393, 1281, 1945, 395, 952, 723, 1269, 111, 1342, 2341, 2536, 657, 2034, 986, 1267, 1607, 2012, 1060, 1631, 628, 1011, 1239, 1982, 2286, 1916, 2136, 1261, 193, 2476, 2360, 2565, 727, 1793, 278, 313, 770, 2600, 2618, 2457, 344, 442, 2474, 1180, 740, 699, 2442, 1503, 121, 1086, 1739, 1601, 1771, 2522, 2063, 347, 689, 2252, 901, 621, 28, 2465, 2319, 958, 1078, 1143, 2250, 2122, 2511, 904, 2613, 1202, 125, 795, 1710, 1652, 1340, 1045, 255, 1975, 1228, 2375, 265, 983, 2225, 492, 1664, 2176, 919, 2251, 408, 1150, 767, 1103, 1227, 1989, 1584, 961, 2615, 1246, 401, 1863, 1351, 1593, 2406, 787, 1054, 1106, 1248, 1238, 1112, 103, 1075, 2033, 267, 698, 2513, 1025, 396, 2440, 1773, 2369, 2616, 1846, 1201, 361, 2468, 1928, 905, 1513, 2008, 2464, 648, 1181, 1830, 424, 2074, 2426, 2291, 675, 383, 623, 1053, 2595, 1055, 1708, 897, 405, 2576, 224, 712, 1936, 175, 2144, 1251, 1126, 1927, 2100, 1094, 2500, 1840, 951, 679, 2418, 629, 429, 2244, 2535, 1328, 738, 1022, 2035, 1335, 92, 1637, 279, 674, 2198, 1539, 1072, 343, 1309, 124, 1545, 2393, 2322, 2492, 372, 398, 2445, 2273, 169, 2018, 889, 848, 1077, 1932, 1344, 2557, 645, 1937, 1042, 1902, 765, 1264, 2301, 2542, 2248, 414, 2043, 1371, 769, 2295, 1993, 61, 2333, 291, 1854, 635, 2611, 1396, 1587, 2247, 1972, 1757, 2117, 2524, 676, 632, 634, 1548, 2447, 2599, 758, 2045, 2276, 2597, 1226, 13, 1507, 1564, 2624, 896, 1440, 1560, 281, 2370, 404, 1751, 1302, 2269, 1730, 1285, 1033, 177, 1749, 1922, 654, 1996, 69, 1941, 1986, 256, 838, 416, 1399, 2166, 411, 702, 1426, 1610, 1405, 1450, 605, 113, 328, 1290, 931, 2056, 106, 1206, 1728, 90, 948, 668, 450, 858, 2589, 2417, 1365, 19, 489, 1029, 2227, 1359, 2293, 2130, 1783, 2147, 2530, 249, 1966, 886, 1096, 503, 1910, 2585, 1380, 2086, 498, 1383, 2131, 213, 1554, 1392, 2289, 827, 2150, 1646, 1969, 2108, 2142, 2388, 128, 1198, 332, 275, 1899, 1811, 1675, 1605, 2094, 430, 176, 1322, 288, 703, 1917, 357, 2308, 957, 2145, 1420, 614, 1692, 1035, 1555, 130, 89, 2394, 2265, 1338, 360, 2242, 16, 2326, 865, 1561, 2627, 1136, 65, 2207, 1904, 1397, 964, 560, 1703, 330, 2427, 2240, 1686, 1817, 2098, 822, 2171, 2441, 2567, 1004, 1502, 2622, 1630, 912, 2448, 1098, 2396, 839, 1988, 2625, 923, 2487, 864, 617, 243, 913, 685, 987, 1881, 508, 1617, 2515, 1196, 615, 1544, 1801, 739, 1571, 2229, 2547, 1015, 2491, 2561, 1016, 1497, 1195, 748, 1411, 1048, 1843, 2302, 2545, 1586, 2374, 1014, 1373, 559, 244, 1994, 2272, 1961, 1232, 1231, 510, 2496, 2190, 2103, 312, 1313, 1297, 1141, 1531, 1090, 1068, 1826, 1521, 509, 651, 202, 752, 1639, 1034, 2436, 1329, 1726, 2231, 438, 761, 2377, 1649, 1597, 2335, 1506, 780, 400, 1230, 2066, 376, 2422, 917, 1082, 1110, 1056, 2090, 72, 2310, 974, 994, 86, 2416, 1701, 925, 1873, 633, 1953, 1211, 1368, 2221, 2467, 2570, 2437, 73, 131, 364, 1049, 1018, 841, 1361, 132, 2451, 2566, 2629, 1914, 2199, 2626, 2149, 1595, 319, 2391, 729, 2246, 434, 2404, 1172, 962, 1523, 2606, 1223, 2550, 1159, 1938, 1915, 2278, 895, 2461, 1253, 1352, 2559, 963, 804, 2093, 2031, 1909, 2359, 730, 2021, 906, 2330, 898, 1321, 1312, 1330, 778, 821, 2539, 1382, 2091, 2256, 355, 2631, 2243, 1670, 378, 2315, 1925, 1666, 2452, 853, 1856, 900, 2446, 2124, 662, 1604, 1645, 725, 1084, 1847, 1705, 247, 2119, 1755, 2587, 17, 327, 1753, 754, 1951, 697, 1541, 598, 565, 1681, 1093, 1013, 346, 1515, 1370, 1262, 2070, 2529, 2053, 2297, 1889, 1973, 371, 2348, 1876, 506, 435, 1277, 1128, 2495, 1327, 2379, 1557, 507, 2519, 2234, 1008, 122, 454, 1074, 1508, 2501, 2158, 1120, 556, 1496, 1244, 1225, 257, 793, 2526, 1551, 2477, 1654, 2512, 1500, 1575, 2395, 263, 381, 782, 2303, 2486, 1602, 696, 1797, 2381, 611, 652, 1855, 929, 813, 181, 718, 1656, 625, 388, 1669, 436, 241, 2279, 2516, 1549, 2554, 918, 1199, 944, 1286, 2125, 687, 1204, 25, 2610, 187, 1935, 31, 2282, 2215, 1337, 2355, 860, 708, 1859, 1298, 2157, 116, 817, 1218, 940, 1559, 893, 2551, 1845, 2101, 600, 27, 2357, 2217, 2608, 317, 894, 1002, 32, 563, 2632, 2083, 66, 558, 862, 2636, 81, 102, 78, 2556, 1759, 796, 2197, 2001, 548, 1921, 1657, 2517, 688, 391, 1363, 2170, 911, 1964, 970, 1636, 870, 1308, 1398, 1907, 1662, 2392, 2061, 2087, 1036, 2390, 1332, 1012, 315, 1809, 1378, 356, 1350, 2175, 173, 1386, 916, 403, 616, 2484, 1512, 934, 1588, 850, 1424, 2097, 18, 1665, 2208, 1282, 1641, 2298, 85, 2398, 253, 2378, 2358, 1672, 774, 637, 2195, 1393, 1724, 709, 252, 1952, 711, 2141, 1704, 1, 2017, 869, 1691, 428, 2537, 2592, 1236, 1421, 1648, 2582, 690, 618, 1930, 2630, 2044, 1877, 501, 2135, 2510, 909, 1535, 96, 1908, 825, 671, 2114, 2455, 969, 1651, 788, 2428, 2167, 1137, 1438, 68, 200, 1192, 2423, 2005, 1634, 715, 691, 1741, 641, 713, 1682, 1177, 1884, 1769, 907, 2594, 1349, 199, 423, 2397, 1169, 831, 1828, 1875, 410, 2431, 1323, 2039, 2288, 673, 737, 1668, 188, 826, 1976, 1063, 1924, 2193, 1590, 1416, 2367, 1040, 493, 1360, 1965, 810, 2621, 2069, 439, 180, 722, 1319, 1428, 62, 2462, 2460, 1579, 2169, 1409, 829, 2400, 2082, 1898, 1390, 1305, 1950, 488, 2572, 1903, 705, 2407, 333, 1697, 1718, 872, 812, 659, 392, 2062, 2546, 1224, 777, 1955, 1495, 2399, 2342, 402, 1815, 1067, 881, 120, 2506, 1623, 1574, 794, 1240, 1064, 1362, 1933, 2245, 1021, 2268, 91, 482, 2480, 1867, 1003, 2347, 1242, 759, 1842, 2494, 2612, 415, 2259, 341, 2059, 484, 1374, 307, 1043, 1339, 2068, 756, 808, 1105, 2604, 1572, 561, 965, 2178, 266, 1208, 1387, 24, 325, 77, 2060, 370, 2509, 1679, 2143, 661, 1991, 2275, 1611, 1614, 185, 1653, 2518, 1052, 1300, 1070, 878, 2054, 1934, 2514, 2481, 2504, 1807, 1273, 1448, 2073, 2316, 1650, 309, 1625, 1667, 14, 716, 1603, 1538, 1655, 2323, 87, 760, 2307, 1085, 1320, 1821, 2334, 2067, 2084, 2337, 775, 1395, 1624, 1347, 2163, 1031, 649, 274, 2580, 1714, 2317, 1188, 859, 921, 365, 1284, 681, 1245, 2055, 308, 956, 1217, 2429, 1777, 295, 2593, 194, 1324, 399, 118, 1839, 2578, 1589, 2521, 222, 1505, 1528, 1853, 119, 1176, 2274, 1289, 1186, 1640, 1292, 1057, 2194, 345, 1138, 2263, 2538, 2280, 1168, 842, 443, 843, 1827, 2057, 2077, 1357, 2531, 875, 1716, 485, 2088, 619, 936, 1304, 2112, 1044, 1556, 828, 2152, 2, 927, 499, 2078, 2133, 2596, 2579, 2581, 1316, 610, 100, 1142, 2408, 3, 976, 2270, 1896, 2607, 2255, 2331, 1974, 2555, 891, 1848, 2314, 1897, 2591, 1894, 2258, 1255, 2277, 1578, 1163, 650, 1234, 1178, 985, 358, 2430, 2187, 1942, 1862, 946, 978, 1761, 320, 2092, 2623, 172, 1293, 1174, 855, 2253, 1114, 1097, 2168, 1166, 1032, 658, 2520, 108, 1963, 602, 786, 2502, 1929, 741, 2015, 324, 655, 157, 1888, 192, 1119, 2192, 2349, 757, 1065, 1954, 1949, 2373, 557, 2424, 2165, 1763, 622, 1066, 449, 115, 1987, 2320, 2201, 2173, 2004, 214, 1676, 2071, 2222, 1835, 726, 832, 2449, 2328, 1868, 892, 1058, 920, 1179, 553, 1173, 1367, 1878, 2470, 922, 1620, 930, 2366, 2204, 1836, 1358, 1454, 2385, 2344, 1265, 1622, 2305, 1315, 1968, 932, 2261, 1524, 1534, 791, 310, 1442, 1643, 1831, 2363, 1047, 746, 1155, 1673, 2435, 422, 2635, 797, 2214, 1041, 873, 1532, 1275, 178, 2115, 390, 933, 2283, 2294, 1425, 104, 562, 1272, 851, 1118, 1413, 1019, 2499, 669, 988, 2339, 766, 710, 1215, 4, 915, 276, 2365, 1958, 322, 293, 2350, 2619, 1844, 2080, 607, 71, 753, 1765, 1943, 368, 750, 683, 1499, 992, 1813, 2189, 2372, 1333, 5, 2351, 1871, 1633, 597, 991, 1642, 1583, 684, 721, 1530, 1258, 2038, 1419, 363, 789, 593, 1612, 2505, 2105, 1979, 1091, 631, 2203, 2266, 447, 2154, 2563, 824, 1837, 490, 2013, 2076, 1092, 101, 2183, 1517, 1536, 2128, 2025, 678, 606, 2137, 2140, 2032, 1037, 820, 437, 603, 2216, 773, 694, 349, 1737, 218, 2064, 1852, 666, 123, 890, 195, 1874, 2586, 638, 502, 2177, 1279, 2425, 979, 2186, 1926, 2011, 2127, 2598, 924, 1632, 1882, 613, 1998, 1997, 1394, 2483, 1684, 1947, 1706, 1214, 1696, 811, 1866, 883, 1348, 1107, 2020, 1695, 1257, 876, 2009, 2568, 1028, 646, 926, 2065, 1880, 1089, 1213, 2237, 348, 2541, 863, 433, 2603, 183, 2281, 1030, 1626, 1596, 2444, 420, 1000, 1569, 397, 857, 2290, 382, 1789, 1346, 2560, 2356, 643, 1967, 2434, 802, 2111, 2376, 389, 1446, 595, 1156, 1132, 0, 1911, 2553, 1294, 724, 384, 599, 26, 2085, 2413, 798, 1372, 2609, 1981, 594, 2386, 1833, 2601, 1027, 1785, 591, 2048, 2340, 1678, 1088, 2220, 2309, 254, 2306, 1295, 747, 1247, 2552, 1102, 366, 2235, 29, 2304, 505, 1430, 2409, 2479, 2051, 1698, 809, 1683, 835, 942, 74, 1823, 2634, 1832, 264, 2287, 2352, 1674, 2414, 1331, 1456, 1712, 2571, 1187, 1117, 1891, 2079, 867, 2389, 1134, 642, 856, 1960, 1051, 1920, 2181, 943, 1369, 2104, 980, 719, 412, 640, 2472, 2095, 220, 182, 2324, 2420, 771, 2558, 2412, 2006, 342, 1185, 2129, 2123, 2439, 2072, 1702, 99, 2200, 1288, 1552, 2134, 612, 1111, 2102, 2402, 1270, 1307, 1343, 1024, 2109, 2223, 1356, 899, 129, 656, 280, 1109, 1825, 10, 672, 318, 1865, 314, 441, 97, 171, 1553, 1940, 1197, 834, 776, 1619, 1779, 1995, 1162, 2107, 968, 871, 292, 636, 1299, 1207, 1573, 734, 1948, 783, 1567, 1627, 2405, 1158, 1613, 2155, 495, 874, 647, 1787, 2602, 1621, 1962, 1857, 479, 1799, 455, 1767, 1688, 1900, 1550, 1893, 406, 306, 706, 1125, 2014, 1355, 1325, 2346, 112, 1618, 1418, 221, 2160, 849, 692, 486, 1237, 554, 592, 2493, 1210, 2174, 2411, 1104, 2146, 735, 2262, 2311, 1547, 2527, 1263, 1977, 1901, 1005, 1913, 601, 1287, 772, 937, 448, 379, 1061, 2226, 1522, 866, 1080, 1685, 608, 127, 984, 64, 2249, 311, 170, 2113, 1381, 242, 833, 1108, 1026, 2184, 2469, 1540, 2180, 2453, 790, 847, 1609, 1918, 425, 2380, 417, 814, 2026, 799, 2209, 1191, 1171, 2564, 105, 1385, 2528, 1526, 982, 1887, 972, 1733, 1939, 1519, 882, 359, 419, 1444, 2361, 386, 996, 367, 1101, 2463, 2410, 2271, 837, 720, 1999, 251, 1212, 1598, 2353, 2614, 1146, 1543, 2415, 1095, 1693, 440, 999, 2202, 114, 1849, 1905, 2299, 1170, 792, 1083, 2489, 1233, 326, 2211, 2003, 1194, 1671, 728, 707, 1364, 1638, 1376, 1978, 1345, 2401, 1518, 88, 908, 2153, 1525, 1254, 63, 1071, 219, 2383, 2219, 1278, 555, 2224, 1069, 1310, 2300, 803, 198, 1563, 966, 20, 2503, 2478, 564, 2022, 1391, 1161, 2110, 1354, 805, 1690, 2148, 362, 977, 1241, 880, 660, 2368, 196, 2182, 2605, 186, 939, 248, 2473, 630, 1038, 844, 1501, 1100, 2164, 2575, 2466, 316, 1576, 1565, 2421, 1050, 15, 407, 768, 1377, 2139, 744, 2027, 2050, 480, 431, 70, 1959, 2159, 2046, 483, 453, 1432, 2138, 2132, 742, 1250, 1116, 1890, 84, 2052, 1059, 902, 1906, 1616, 1076, 2343, 1130, 914, 2196, 2228, 2047, 1841, 779, 2096, 2327, 2019, 1266, 2028, 941, 421, 2238, 2433, 2354, 387, 215, 1183, 1694, 1167, 819, 1113, 928, 1249, 2002, 2583, 1434, 497, 1546, 481, 1992, 331, 1216, 1334, 1838, 1829, 1775, 1260, 626, 1317, 1318, 2617, 2030, 1271, 997, 1581, 755, 352, 2260, 885, 340, 1388, 456, 2588, 2544, 1209, 949, 1046, 884, 1190, 1436, 823, 1879, 500, 2029, 1680, 277, 1219, 816, 350, 2490, 990, 1803, 1205, 2081, 1274, 764, 2338, 989, 2590, 733, 800, 1366, 2041, 2024, 1087, 1122, 2284, 664, 947, 1677, 1200, 110, 1600, 1870, 1663, 1203, 2089, 1985, 323, 2230, 2236, 714, 93, 751, 9, 2458, 1243, 2403, 620, 861, 246, 1520, 887, 1822, 1511, 836, 717, 167, 1819, 784, 815, 2042, 1594, 1659, 1401, 1189, 2218, 2454, 1280, 1389, 1153, 184, 2285, 1375, 2482, 496, 2010, 854, 1628, 2058, 217, 504, 2485, 191, 1886, 23, 680, 2172, 2075, 2387, 2488, 168, 749, 731, 1314, 174, 2562, 2241, 1533, 2040, 1582, 1001, 1700, 2577, 596, 1990, 1861, 2540, 21, 1384, 1115, 2156, 701, 1892, 1608, 2188, 80, 609, 1687, 2364, 409, 2036, 1010, 935, 1017, 2508, 1984, 2185, 2471, 1268, 1509, 1850, 704, 2532, 1745, 1020, 2120, 1336, 981, 2497, 83, 2321, 2116, 1422, 624, 2233, 1164, 179, 663, 30, 975, 1615, 1353, 830, 427, 1858, 1851, 290, 781, 2049, 351, 1592, 995, 321, 2332, 653, 2213, 33, 1743, 1883, 879, 245, 971, 1124, 1566, 700, 2345, 2548, 1306, 1635, 959, 2121, 2232, 2179, 2432, 2450, 2212, 736, 806, 2329, 1510, 1504, 2443, 223, 2533, 2569, 2523, 2498, 2023, 1256, 1956, 418, 1957, 1452, 2264, 1570, 763, 2475, 852, 2543, 2205, 2633, 1860, 2210, 79, 2257, 2267, 1276, 1580, 1568, 938, 745, 1606, 1081, 117, 1931, 212, 1039, 22, 686, 967, 1919, 1229, 1222, 1160, 2573, 2456, 109, 1970, 377, 385, 818, 94, 2206, 6, 1872, 695, 1699, 1144, 329, 1629, 107, 1599, 910, 1661, 446, 1885, 394, 868, 2362, 732, 845, 846, 1147, 1577, 76, 1379, 2574, 2239, 960, 2382, 8, 12, 945, 67, 1252, 82, 1834, 1175, 451, 1326, 1781, 2384, 1923, 2620, 1983, 807, 1283, 34, 2318, 1558, 1869, 491, 955, 604, 2549, 2312, 1235, 1944, 1946, 2151, 2191, 1542, 1154, 95, 877, 993, 1184, 1895, 1747, 2099, 1562, 2628],
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

pub(super) static KEYSYM32_TO_CHAR: &[Keysym32Char] = &[
    Keysym32Char {
        keysym: 0x10081200,
        char: '0',
    },
    Keysym32Char {
        keysym: 0x10081201,
        char: '1',
    },
    Keysym32Char {
        keysym: 0x10081202,
        char: '2',
    },
    Keysym32Char {
        keysym: 0x10081203,
        char: '3',
    },
    Keysym32Char {
        keysym: 0x10081204,
        char: '4',
    },
    Keysym32Char {
        keysym: 0x10081205,
        char: '5',
    },
    Keysym32Char {
        keysym: 0x10081206,
        char: '6',
    },
    Keysym32Char {
        keysym: 0x10081207,
        char: '7',
    },
    Keysym32Char {
        keysym: 0x10081208,
        char: '8',
    },
    Keysym32Char {
        keysym: 0x10081209,
        char: '9',
    },
    Keysym32Char {
        keysym: 0x1008120a,
        char: '*',
    },
    Keysym32Char {
        keysym: 0x1008120b,
        char: '#',
    },
];

pub(super) static CHAR_TO_BESPOKE_IDX: PhfMap<char, u16> = PhfMap {
    key: 10656210946825422025,
    disps: &[(0, 334), (0, 73), (0, 4), (0, 42), (0, 6), (0, 0), (0, 7), (0, 4), (0, 0), (0, 1), (0, 0), (0, 2), (0, 1), (0, 2), (0, 101), (0, 19), (0, 6), (0, 104), (0, 2), (0, 0), (0, 16), (0, 15), (0, 2), (0, 79), (0, 9), (0, 3), (0, 22), (0, 64), (0, 9), (0, 1), (0, 69), (0, 6), (0, 0), (0, 116), (0, 4), (0, 11), (0, 30), (0, 65), (0, 0), (0, 101), (0, 4), (0, 42), (0, 41), (0, 5), (0, 2), (0, 94), (0, 168), (0, 21), (0, 57), (0, 31), (0, 13), (0, 8), (0, 1), (0, 191), (0, 69), (0, 188), (0, 153), (0, 10), (0, 10), (0, 17), (0, 17), (0, 5), (0, 164), (0, 5), (0, 12), (0, 74), (0, 13), (0, 0), (0, 14), (0, 0), (0, 0), (0, 283), (0, 12), (0, 32), (0, 73), (0, 0), (0, 0), (0, 1), (0, 4), (0, 0), (0, 0), (0, 3), (0, 0), (0, 99), (0, 1), (0, 0), (0, 61), (0, 33), (0, 39), (0, 31), (0, 10), (0, 0), (0, 0), (0, 14), (0, 1), (0, 0), (0, 1), (0, 7), (0, 24), (0, 33), (0, 87), (0, 14), (0, 23), (0, 1), (0, 1), (0, 2), (0, 144), (0, 95), (0, 4), (0, 10), (0, 224), (0, 108), (0, 6), (0, 101), (0, 1), (0, 19), (0, 2), (0, 8), (0, 151), (0, 0), (0, 88), (0, 69), (0, 192), (0, 54), (0, 190), (0, 77), (0, 0), (0, 8), (0, 284), (0, 33), (0, 86), (0, 771), (0, 150), (0, 117), (0, 187), (0, 233), (0, 656), (0, 54), (0, 13), (0, 434), (0, 0), (0, 103), (0, 29), (0, 22), (0, 362), (0, 59), (0, 4), (0, 141), (0, 23), (0, 0), (0, 595), (0, 339), (0, 173), (0, 50), (0, 1), (0, 1), (0, 3), (0, 770), (0, 38), (0, 396), (0, 66), (0, 1), (0, 1), (0, 10), (0, 0), (0, 294), (0, 157), (0, 0), (0, 10), (0, 16), (0, 652), (0, 212), (0, 115), (0, 2), (0, 728), (0, 0), (0, 1), (0, 10), (0, 4), (0, 44), (0, 0), (0, 0), (0, 105), (0, 2), (0, 2), (0, 4), (0, 203), (0, 19), (0, 0), (0, 47), (0, 1), (0, 137), (0, 32), (0, 303), (0, 146), (0, 98), (0, 0), (1, 164), (1, 0), (0, 43), (0, 3), (0, 45), (0, 0), (0, 31), (0, 69), (0, 39), (0, 181), (0, 2), (0, 690), (0, 184), (0, 7), (0, 29), (0, 17), (0, 26), (0, 3), (0, 21), (0, 718), (1, 54), (1, 648), (0, 38), (5, 537), (0, 320), (0, 615), (0, 0), (3, 147), (0, 613), (0, 248), (2, 163), (0, 22), (0, 10), (0, 347), (0, 38), (0, 71), (0, 15), (0, 0), (0, 143), (0, 39), (0, 14), (0, 57), (0, 670), (0, 1), (1, 651), (0, 282), (0, 18), (2, 432), (0, 89), (0, 2), (0, 88), (0, 3), (0, 0), (0, 118), (0, 386), (2, 258), (1, 763), (4, 766), (6, 661)],
    map: &[656, 107, 383, 574, 270, 175, 188, 682, 820, 554, 957, 307, 338, 411, 880, 727, 987, 399, 54, 945, 846, 517, 1000, 1, 851, 979, 606, 630, 600, 565, 76, 871, 343, 741, 124, 49, 892, 426, 721, 974, 19, 787, 571, 749, 665, 408, 627, 118, 374, 720, 347, 250, 937, 657, 492, 485, 458, 10, 907, 610, 755, 859, 661, 752, 391, 427, 73, 197, 538, 242, 108, 1010, 731, 165, 252, 489, 34, 8, 450, 510, 273, 139, 303, 432, 444, 251, 735, 309, 247, 227, 221, 1196, 241, 439, 51, 919, 588, 718, 525, 739, 711, 147, 27, 85, 938, 90, 493, 535, 753, 238, 65, 580, 451, 876, 392, 473, 915, 835, 785, 253, 594, 341, 101, 172, 15, 302, 799, 278, 923, 233, 185, 962, 530, 484, 888, 81, 501, 317, 558, 585, 503, 951, 59, 231, 24, 201, 889, 533, 319, 176, 217, 520, 351, 692, 640, 405, 545, 282, 38, 906, 189, 115, 220, 717, 946, 28, 905, 772, 136, 291, 702, 72, 494, 335, 1398, 852, 700, 703, 645, 516, 203, 109, 400, 623, 765, 843, 967, 976, 194, 671, 435, 453, 368, 980, 1006, 498, 649, 263, 568, 801, 20, 289, 740, 959, 380, 62, 754, 286, 582, 643, 603, 149, 984, 680, 614, 413, 22, 816, 202, 56, 344, 567, 96, 3, 283, 576, 793, 651, 459, 689, 881, 381, 663, 893, 882, 230, 410, 660, 860, 157, 144, 487, 832, 515, 143, 894, 169, 476, 513, 981, 887, 607, 762, 111, 724, 873, 546, 324, 812, 758, 293, 87, 870, 781, 868, 353, 589, 355, 64, 334, 348, 931, 631, 514, 792, 591, 462, 102, 82, 12, 310, 909, 924, 401, 123, 948, 130, 505, 91, 620, 508, 357, 21, 865, 628, 939, 491, 511, 342, 199, 339, 294, 274, 652, 840, 45, 521, 822, 989, 902, 653, 647, 215, 288, 861, 212, 534, 708, 921, 78, 611, 37, 746, 183, 133, 900, 285, 442, 53, 1189, 766, 956, 190, 495, 164, 696, 416, 968, 224, 837, 750, 29, 764, 394, 997, 596, 802, 17, 769, 174, 145, 667, 615, 187, 532, 1002, 691, 208, 370, 560, 751, 482, 292, 916, 216, 953, 277, 229, 681, 557, 883, 434, 475, 257, 809, 479, 407, 332, 784, 573, 854, 690, 932, 103, 191, 969, 30, 35, 308, 467, 760, 255, 705, 138, 356, 685, 982, 325, 402, 234, 254, 605, 540, 454, 112, 367, 232, 845, 75, 117, 196, 431, 549, 104, 608, 878, 536, 624, 207, 206, 867, 58, 84, 683, 986, 168, 480, 961, 419, 152, 316, 728, 457, 61, 895, 284, 259, 912, 862, 421, 940, 158, 92, 236, 803, 358, 579, 578, 146, 650, 933, 509, 488, 122, 570, 918, 295, 697, 267, 839, 345, 430, 5, 757, 768, 260, 593, 599, 834, 437, 996, 625, 814, 616, 320, 80, 817, 113, 896, 668, 518, 826, 89, 50, 23, 218, 925, 161, 744, 547, 386, 522, 581, 44, 838, 125, 779, 237, 584, 465, 36, 14, 428, 911, 736, 701, 162, 875, 941, 93, 529, 276, 120, 359, 613, 223, 245, 583, 553, 209, 395, 182, 975, 304, 601, 992, 179, 885, 523, 244, 963, 204, 559, 301, 693, 670, 786, 632, 55, 68, 677, 994, 710, 904, 366, 655, 240, 375, 958, 1005, 481, 184, 1191, 842, 167, 95, 695, 97, 382, 738, 412, 326, 855, 497, 279, 715, 995, 927, 805, 819, 2, 926, 884, 423, 729, 192, 336, 105, 684, 328, 519, 69, 983, 171, 153, 132, 63, 31, 477, 456, 694, 178, 950, 763, 180, 658, 712, 213, 524, 142, 362, 872, 575, 999, 404, 934, 722, 971, 32, 265, 119, 821, 572, 506, 271, 747, 321, 388, 811, 502, 300, 512, 662, 897, 114, 646, 890, 564, 543, 550, 140, 6, 198, 305, 372, 687, 707, 732, 528, 828, 11, 771, 504, 211, 970, 869, 1001, 337, 77, 644, 551, 641, 39, 743, 418, 248, 287, 908, 346, 609, 1007, 733, 942, 296, 371, 94, 947, 719, 988, 815, 129, 314, 935, 539, 490, 669, 377, 848, 886, 7, 396, 269, 349, 389, 66, 311, 836, 563, 595, 299, 898, 648, 964, 52, 173, 86, 794, 46, 313, 877, 258, 318, 955, 704, 360, 991, 847, 555, 25, 417, 163, 952, 40, 1188, 393, 384, 856, 249, 177, 205, 148, 350, 361, 406, 913, 16, 642, 928, 672, 186, 312, 849, 617, 998, 409, 70, 943, 424, 531, 246, 448, 460, 378, 699, 844, 767, 977, 329, 397, 474, 106, 385, 761, 363, 127, 561, 415, 298, 725, 116, 98, 369, 33, 972, 181, 569, 43, 57, 759, 154, 1192, 315, 471, 866, 364, 214, 306, 742, 659, 137, 499, 507, 770, 373, 552, 857, 151, 745, 195, 71, 829, 542, 4, 83, 679, 698, 1009, 420, 541, 598, 810, 929, 853, 219, 586, 891, 155, 48, 141, 879, 688, 414, 433, 425, 1008, 379, 222, 526, 47, 340, 446, 42, 60, 874, 813, 272, 936, 436, 88, 824, 686, 973, 590, 917, 901, 243, 469, 920, 390, 676, 748, 327, 235, 791, 18, 864, 99, 79, 13, 795, 621, 226, 297, 592, 612, 833, 483, 965, 200, 807, 121, 604, 556, 899, 275, 949, 160, 1004, 664, 960, 629, 486, 587, 734, 990, 228, 156, 403, 170, 463, 910, 678, 440, 756, 944, 674, 1003, 709, 239, 131, 74, 135, 262, 134, 850, 675, 398, 903, 783, 978, 331, 268, 496, 429, 966, 841, 26, 914, 566, 985, 597, 562, 730, 544, 537, 993, 290, 100, 673, 654, 41, 797, 256, 863, 387, 1190, 706, 376, 225, 330, 266, 210, 261, 954, 280, 323, 264, 500, 527, 858, 452, 626, 713, 922, 930, 830],
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
        keysym_or_definitive_idx: 0x0000fe8e,
        name_start: 11997,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe90,
        name_start: 12012,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe91,
        name_start: 12024,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe92,
        name_start: 12046,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe93,
        name_start: 12068,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea0,
        name_start: 12091,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea1,
        name_start: 12093,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea2,
        name_start: 12095,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea3,
        name_start: 12097,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea4,
        name_start: 12100,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea5,
        name_start: 12103,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed0,
        name_start: 12106,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed1,
        name_start: 12126,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed2,
        name_start: 12145,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed4,
        name_start: 12164,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed5,
        name_start: 12183,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee0,
        name_start: 12199,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee1,
        name_start: 12211,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee2,
        name_start: 12224,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee3,
        name_start: 12234,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee4,
        name_start: 12246,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee5,
        name_start: 12260,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee6,
        name_start: 12275,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee7,
        name_start: 12291,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee8,
        name_start: 12308,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee9,
        name_start: 12327,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feea,
        name_start: 12342,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feeb,
        name_start: 12357,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feec,
        name_start: 12372,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feed,
        name_start: 12387,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feee,
        name_start: 12402,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feef,
        name_start: 12423,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef0,
        name_start: 12440,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef1,
        name_start: 12457,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef2,
        name_start: 12474,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef3,
        name_start: 12491,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef4,
        name_start: 12508,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef5,
        name_start: 12525,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef6,
        name_start: 12538,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef7,
        name_start: 12551,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef8,
        name_start: 12564,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef9,
        name_start: 12577,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefa,
        name_start: 12595,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefb,
        name_start: 12613,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefc,
        name_start: 12632,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefd,
        name_start: 12651,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff08,
        name_start: 12664,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff09,
        name_start: 12673,
        name_len: 3,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff0a,
        name_start: 12676,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff0b,
        name_start: 12684,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff0d,
        name_start: 12689,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff13,
        name_start: 12695,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff14,
        name_start: 12700,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff15,
        name_start: 12711,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff1b,
        name_start: 12718,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff20,
        name_start: 12724,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ad,
        name_start: 12733,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff21,
        name_start: 12743,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff22,
        name_start: 12748,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff23,
        name_start: 12756,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b1,
        name_start: 12767,
        name_len: 6,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff24,
        name_start: 12773,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff25,
        name_start: 12779,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff26,
        name_start: 12787,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff27,
        name_start: 12795,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff28,
        name_start: 12812,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff29,
        name_start: 12819,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2a,
        name_start: 12826,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2b,
        name_start: 12841,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2c,
        name_start: 12848,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2d,
        name_start: 12854,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2e,
        name_start: 12863,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2f,
        name_start: 12873,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff30,
        name_start: 12883,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff31,
        name_start: 12894,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff32,
        name_start: 12900,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff33,
        name_start: 12912,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff34,
        name_start: 12922,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff35,
        name_start: 12934,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff36,
        name_start: 12945,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff37,
        name_start: 12958,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c6,
        name_start: 12967,
        name_len: 12,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c6,
        name_start: 12979,
        name_len: 16,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff38,
        name_start: 12995,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff39,
        name_start: 13008,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3a,
        name_start: 13020,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3b,
        name_start: 13035,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3c,
        name_start: 13051,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cd,
        name_start: 13066,
        name_len: 22,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3d,
        name_start: 13088,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cf,
        name_start: 13105,
        name_len: 8,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cf,
        name_start: 13113,
        name_len: 24,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3e,
        name_start: 13137,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d2,
        name_start: 13154,
        name_len: 8,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d2,
        name_start: 13162,
        name_len: 24,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3f,
        name_start: 13186,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff50,
        name_start: 13200,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff51,
        name_start: 13204,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff52,
        name_start: 13208,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff53,
        name_start: 13210,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff54,
        name_start: 13215,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff55,
        name_start: 13219,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004db,
        name_start: 13224,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004db,
        name_start: 13231,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff56,
        name_start: 13240,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004de,
        name_start: 13244,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004de,
        name_start: 13253,
        name_len: 11,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff57,
        name_start: 13264,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff58,
        name_start: 13267,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff60,
        name_start: 13272,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff61,
        name_start: 13278,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004e4,
        name_start: 13283,
        name_len: 15,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff62,
        name_start: 13298,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff63,
        name_start: 13305,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff65,
        name_start: 13311,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004e8,
        name_start: 13315,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff66,
        name_start: 13322,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ea,
        name_start: 13326,
        name_len: 8,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff67,
        name_start: 13334,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff68,
        name_start: 13338,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ed,
        name_start: 13342,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff69,
        name_start: 13349,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ef,
        name_start: 13355,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff6a,
        name_start: 13362,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff6b,
        name_start: 13366,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff7e,
        name_start: 13371,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f3,
        name_start: 13386,
        name_len: 11,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f3,
        name_start: 13397,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f3,
        name_start: 13410,
        name_len: 11,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f3,
        name_start: 13421,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f3,
        name_start: 13434,
        name_len: 12,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f3,
        name_start: 13446,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f3,
        name_start: 13459,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f3,
        name_start: 13472,
        name_len: 11,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff7f,
        name_start: 13483,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff80,
        name_start: 13491,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff89,
        name_start: 13499,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff8d,
        name_start: 13505,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff91,
        name_start: 13513,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff92,
        name_start: 13518,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff93,
        name_start: 13523,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff94,
        name_start: 13528,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff95,
        name_start: 13533,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff96,
        name_start: 13540,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff97,
        name_start: 13547,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff98,
        name_start: 13552,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff99,
        name_start: 13560,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9a,
        name_start: 13567,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000509,
        name_start: 13575,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9b,
        name_start: 13585,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000050b,
        name_start: 13592,
        name_len: 12,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9c,
        name_start: 13604,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9d,
        name_start: 13610,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9e,
        name_start: 13618,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9f,
        name_start: 13627,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffaa,
        name_start: 13636,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffab,
        name_start: 13647,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffac,
        name_start: 13653,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffad,
        name_start: 13665,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffae,
        name_start: 13676,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffaf,
        name_start: 13686,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb0,
        name_start: 13695,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb1,
        name_start: 13699,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb2,
        name_start: 13703,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb3,
        name_start: 13707,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb4,
        name_start: 13711,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb5,
        name_start: 13715,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb6,
        name_start: 13719,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb7,
        name_start: 13723,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb8,
        name_start: 13727,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb9,
        name_start: 13731,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffbd,
        name_start: 13735,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffbe,
        name_start: 13743,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffbf,
        name_start: 13745,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc0,
        name_start: 13747,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc1,
        name_start: 13749,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc2,
        name_start: 13751,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc3,
        name_start: 13753,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc4,
        name_start: 13755,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc5,
        name_start: 13757,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc6,
        name_start: 13759,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc7,
        name_start: 13761,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc8,
        name_start: 13764,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000052c,
        name_start: 13767,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc9,
        name_start: 13769,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000052e,
        name_start: 13772,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffca,
        name_start: 13774,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000530,
        name_start: 13777,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcb,
        name_start: 13779,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000532,
        name_start: 13782,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcc,
        name_start: 13784,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000534,
        name_start: 13787,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcd,
        name_start: 13789,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000536,
        name_start: 13792,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffce,
        name_start: 13794,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000538,
        name_start: 13797,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcf,
        name_start: 13799,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000053a,
        name_start: 13802,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd0,
        name_start: 13804,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000053c,
        name_start: 13807,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd1,
        name_start: 13809,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000053e,
        name_start: 13812,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd2,
        name_start: 13815,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000540,
        name_start: 13818,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd3,
        name_start: 13820,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000542,
        name_start: 13823,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd4,
        name_start: 13825,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000544,
        name_start: 13828,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd5,
        name_start: 13830,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000546,
        name_start: 13833,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd6,
        name_start: 13835,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000548,
        name_start: 13838,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd7,
        name_start: 13840,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000054a,
        name_start: 13843,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd8,
        name_start: 13845,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000054c,
        name_start: 13848,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd9,
        name_start: 13850,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000054e,
        name_start: 13853,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffda,
        name_start: 13855,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000550,
        name_start: 13858,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdb,
        name_start: 13860,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000552,
        name_start: 13863,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdc,
        name_start: 13866,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000554,
        name_start: 13869,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdd,
        name_start: 13872,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000556,
        name_start: 13875,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffde,
        name_start: 13878,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000558,
        name_start: 13881,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdf,
        name_start: 13884,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000055a,
        name_start: 13887,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe0,
        name_start: 13890,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000055c,
        name_start: 13893,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe1,
        name_start: 13896,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe2,
        name_start: 13903,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe3,
        name_start: 13910,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe4,
        name_start: 13919,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe5,
        name_start: 13928,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe6,
        name_start: 13937,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe7,
        name_start: 13947,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe8,
        name_start: 13953,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe9,
        name_start: 13959,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffea,
        name_start: 13964,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffeb,
        name_start: 13969,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffec,
        name_start: 13976,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffed,
        name_start: 13983,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffee,
        name_start: 13990,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff1,
        name_start: 13997,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff2,
        name_start: 14010,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff3,
        name_start: 14023,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff4,
        name_start: 14036,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff5,
        name_start: 14049,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff6,
        name_start: 14062,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff7,
        name_start: 14075,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff8,
        name_start: 14088,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff9,
        name_start: 14101,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fffa,
        name_start: 14114,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffff,
        name_start: 14128,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00ffffff,
        name_start: 14134,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100012c,
        name_start: 14144,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100012d,
        name_start: 14150,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000174,
        name_start: 14156,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000175,
        name_start: 14167,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000176,
        name_start: 14178,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000177,
        name_start: 14189,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100018f,
        name_start: 14200,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100019f,
        name_start: 14205,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001a0,
        name_start: 14212,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001a1,
        name_start: 14217,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001af,
        name_start: 14222,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b0,
        name_start: 14227,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b5,
        name_start: 14232,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b6,
        name_start: 14239,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b7,
        name_start: 14246,
        name_len: 3,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001d1,
        name_start: 14249,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001d2,
        name_start: 14255,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001e6,
        name_start: 14261,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001e7,
        name_start: 14267,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000259,
        name_start: 14273,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000275,
        name_start: 14278,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000292,
        name_start: 14285,
        name_len: 3,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000300,
        name_start: 14288,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000301,
        name_start: 14303,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000303,
        name_start: 14318,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000309,
        name_start: 14333,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000323,
        name_start: 14347,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000492,
        name_start: 14365,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000493,
        name_start: 14381,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000496,
        name_start: 14397,
        name_len: 22,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000497,
        name_start: 14419,
        name_len: 22,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049a,
        name_start: 14441,
        name_len: 21,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049b,
        name_start: 14462,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049c,
        name_start: 14483,
        name_len: 22,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049d,
        name_start: 14505,
        name_len: 22,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004a2,
        name_start: 14527,
        name_len: 21,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004a3,
        name_start: 14548,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ae,
        name_start: 14569,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004af,
        name_start: 14588,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b0,
        name_start: 14607,
        name_len: 23,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b1,
        name_start: 14630,
        name_len: 23,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b2,
        name_start: 14653,
        name_len: 21,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b3,
        name_start: 14674,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b6,
        name_start: 14695,
        name_len: 22,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b7,
        name_start: 14717,
        name_len: 22,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b8,
        name_start: 14739,
        name_len: 23,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b9,
        name_start: 14762,
        name_len: 23,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ba,
        name_start: 14785,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004bb,
        name_start: 14798,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004d8,
        name_start: 14811,
        name_len: 14,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004d9,
        name_start: 14825,
        name_len: 14,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e2,
        name_start: 14839,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e3,
        name_start: 14856,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e8,
        name_start: 14873,
        name_len: 14,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e9,
        name_start: 14887,
        name_len: 14,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ee,
        name_start: 14901,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ef,
        name_start: 14918,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000531,
        name_start: 14935,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000532,
        name_start: 14947,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000533,
        name_start: 14959,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000534,
        name_start: 14971,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000535,
        name_start: 14982,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000536,
        name_start: 14995,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000537,
        name_start: 15006,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000538,
        name_start: 15016,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000539,
        name_start: 15027,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053a,
        name_start: 15038,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053b,
        name_start: 15050,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053c,
        name_start: 15062,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053d,
        name_start: 15075,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053e,
        name_start: 15087,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053f,
        name_start: 15099,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000540,
        name_start: 15111,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000541,
        name_start: 15122,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000542,
        name_start: 15134,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000543,
        name_start: 15147,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000544,
        name_start: 15160,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000545,
        name_start: 15172,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000546,
        name_start: 15183,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000547,
        name_start: 15194,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000548,
        name_start: 15206,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000549,
        name_start: 15217,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054a,
        name_start: 15229,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054b,
        name_start: 15240,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054c,
        name_start: 15251,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054d,
        name_start: 15262,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054e,
        name_start: 15273,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054f,
        name_start: 15285,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000550,
        name_start: 15298,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000551,
        name_start: 15309,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000552,
        name_start: 15321,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000553,
        name_start: 15334,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000554,
        name_start: 15347,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000555,
        name_start: 15358,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000556,
        name_start: 15368,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055a,
        name_start: 15379,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055b,
        name_start: 15398,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d8,
        name_start: 15413,
        name_len: 15,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055c,
        name_start: 15428,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005da,
        name_start: 15443,
        name_len: 15,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055d,
        name_start: 15458,
        name_len: 24,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005dc,
        name_start: 15482,
        name_len: 12,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055e,
        name_start: 15494,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005de,
        name_start: 15511,
        name_len: 15,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000561,
        name_start: 15526,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000562,
        name_start: 15538,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000563,
        name_start: 15550,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000564,
        name_start: 15562,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000565,
        name_start: 15573,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000566,
        name_start: 15586,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000567,
        name_start: 15597,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000568,
        name_start: 15607,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000569,
        name_start: 15618,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056a,
        name_start: 15629,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056b,
        name_start: 15641,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056c,
        name_start: 15653,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056d,
        name_start: 15666,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056e,
        name_start: 15678,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056f,
        name_start: 15690,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000570,
        name_start: 15702,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000571,
        name_start: 15713,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000572,
        name_start: 15725,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000573,
        name_start: 15738,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000574,
        name_start: 15751,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000575,
        name_start: 15763,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000576,
        name_start: 15774,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000577,
        name_start: 15785,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000578,
        name_start: 15797,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000579,
        name_start: 15808,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057a,
        name_start: 15820,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057b,
        name_start: 15831,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057c,
        name_start: 15842,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057d,
        name_start: 15853,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057e,
        name_start: 15864,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057f,
        name_start: 15876,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000580,
        name_start: 15889,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000581,
        name_start: 15900,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000582,
        name_start: 15912,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000583,
        name_start: 15925,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000584,
        name_start: 15938,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000585,
        name_start: 15949,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000586,
        name_start: 15959,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000587,
        name_start: 15970,
        name_len: 20,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000589,
        name_start: 15990,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000607,
        name_start: 16008,
        name_len: 17,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100058a,
        name_start: 16025,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000609,
        name_start: 16040,
        name_len: 17,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000653,
        name_start: 16057,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000654,
        name_start: 16075,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000655,
        name_start: 16093,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000660,
        name_start: 16111,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000661,
        name_start: 16119,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000662,
        name_start: 16127,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000663,
        name_start: 16135,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000664,
        name_start: 16143,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000665,
        name_start: 16151,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000666,
        name_start: 16159,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000667,
        name_start: 16167,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000668,
        name_start: 16175,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000669,
        name_start: 16183,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100066a,
        name_start: 16191,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000670,
        name_start: 16205,
        name_len: 23,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000679,
        name_start: 16228,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100067e,
        name_start: 16239,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000686,
        name_start: 16249,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000688,
        name_start: 16261,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000691,
        name_start: 16272,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000698,
        name_start: 16283,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006a4,
        name_start: 16293,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006a9,
        name_start: 16303,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006af,
        name_start: 16315,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006ba,
        name_start: 16325,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006be,
        name_start: 16343,
        name_len: 22,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006c1,
        name_start: 16365,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006cc,
        name_start: 16380,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000626,
        name_start: 16389,
        name_len: 16,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006d2,
        name_start: 16405,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006d4,
        name_start: 16421,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f0,
        name_start: 16436,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f1,
        name_start: 16443,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f2,
        name_start: 16450,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f3,
        name_start: 16457,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f4,
        name_start: 16464,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f5,
        name_start: 16471,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f6,
        name_start: 16478,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f7,
        name_start: 16485,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f8,
        name_start: 16492,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f9,
        name_start: 16499,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d82,
        name_start: 16506,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d83,
        name_start: 16513,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d85,
        name_start: 16520,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d86,
        name_start: 16526,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d87,
        name_start: 16533,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d88,
        name_start: 16540,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d89,
        name_start: 16548,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8a,
        name_start: 16554,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8b,
        name_start: 16561,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8c,
        name_start: 16567,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8d,
        name_start: 16574,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8e,
        name_start: 16581,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8f,
        name_start: 16589,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d90,
        name_start: 16596,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d91,
        name_start: 16604,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d92,
        name_start: 16610,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d93,
        name_start: 16617,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d94,
        name_start: 16624,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d95,
        name_start: 16630,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d96,
        name_start: 16637,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9a,
        name_start: 16644,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9b,
        name_start: 16651,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9c,
        name_start: 16659,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9d,
        name_start: 16666,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9e,
        name_start: 16674,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9f,
        name_start: 16682,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da0,
        name_start: 16690,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da1,
        name_start: 16697,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da2,
        name_start: 16705,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da3,
        name_start: 16712,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da4,
        name_start: 16720,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da5,
        name_start: 16728,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da6,
        name_start: 16737,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da7,
        name_start: 16745,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da8,
        name_start: 16753,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da9,
        name_start: 16762,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000daa,
        name_start: 16770,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dab,
        name_start: 16779,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dac,
        name_start: 16787,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dad,
        name_start: 16796,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dae,
        name_start: 16804,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000daf,
        name_start: 16813,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db0,
        name_start: 16821,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db1,
        name_start: 16830,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db3,
        name_start: 16837,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db4,
        name_start: 16846,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db5,
        name_start: 16853,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db6,
        name_start: 16861,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db7,
        name_start: 16868,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db8,
        name_start: 16876,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db9,
        name_start: 16883,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dba,
        name_start: 16891,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dbb,
        name_start: 16898,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dbd,
        name_start: 16905,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc0,
        name_start: 16912,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc1,
        name_start: 16919,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc2,
        name_start: 16927,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc3,
        name_start: 16936,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc4,
        name_start: 16943,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc5,
        name_start: 16950,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc6,
        name_start: 16958,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dca,
        name_start: 16965,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dcf,
        name_start: 16972,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd0,
        name_start: 16980,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd1,
        name_start: 16988,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd2,
        name_start: 16997,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd3,
        name_start: 17004,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd4,
        name_start: 17012,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd6,
        name_start: 17019,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd8,
        name_start: 17027,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd9,
        name_start: 17035,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dda,
        name_start: 17042,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddb,
        name_start: 17050,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddc,
        name_start: 17058,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddd,
        name_start: 17065,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dde,
        name_start: 17073,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddf,
        name_start: 17081,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000df2,
        name_start: 17089,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000df3,
        name_start: 17098,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000df4,
        name_start: 17107,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d0,
        name_start: 17122,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d1,
        name_start: 17133,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d2,
        name_start: 17145,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d3,
        name_start: 17157,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d4,
        name_start: 17169,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d5,
        name_start: 17180,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d6,
        name_start: 17192,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d7,
        name_start: 17204,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d8,
        name_start: 17216,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d9,
        name_start: 17227,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010da,
        name_start: 17239,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010db,
        name_start: 17251,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010dc,
        name_start: 17263,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010dd,
        name_start: 17275,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010de,
        name_start: 17286,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010df,
        name_start: 17298,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e0,
        name_start: 17311,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e1,
        name_start: 17323,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e2,
        name_start: 17335,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e3,
        name_start: 17347,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e4,
        name_start: 17358,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e5,
        name_start: 17371,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e6,
        name_start: 17384,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e7,
        name_start: 17397,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e8,
        name_start: 17409,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e9,
        name_start: 17422,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ea,
        name_start: 17435,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010eb,
        name_start: 17447,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ec,
        name_start: 17459,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ed,
        name_start: 17471,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ee,
        name_start: 17484,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ef,
        name_start: 17496,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f0,
        name_start: 17509,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f1,
        name_start: 17521,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f2,
        name_start: 17532,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f3,
        name_start: 17544,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f4,
        name_start: 17555,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f5,
        name_start: 17567,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f6,
        name_start: 17579,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e02,
        name_start: 17590,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e03,
        name_start: 17599,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e0a,
        name_start: 17608,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e0b,
        name_start: 17617,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e1e,
        name_start: 17626,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e1f,
        name_start: 17635,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e36,
        name_start: 17644,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e37,
        name_start: 17653,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e40,
        name_start: 17662,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e41,
        name_start: 17671,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e56,
        name_start: 17680,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e57,
        name_start: 17689,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e60,
        name_start: 17698,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e61,
        name_start: 17707,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e6a,
        name_start: 17716,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e6b,
        name_start: 17725,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e80,
        name_start: 17734,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e81,
        name_start: 17740,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e82,
        name_start: 17746,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e83,
        name_start: 17752,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e84,
        name_start: 17758,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e85,
        name_start: 17768,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e8a,
        name_start: 17778,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e8b,
        name_start: 17787,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e9e,
        name_start: 17796,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea0,
        name_start: 17802,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea1,
        name_start: 17811,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea2,
        name_start: 17820,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea3,
        name_start: 17825,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea4,
        name_start: 17830,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea5,
        name_start: 17846,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea6,
        name_start: 17862,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea7,
        name_start: 17878,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea8,
        name_start: 17894,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea9,
        name_start: 17909,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eaa,
        name_start: 17924,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eab,
        name_start: 17940,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eac,
        name_start: 17956,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ead,
        name_start: 17975,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eae,
        name_start: 17994,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eaf,
        name_start: 18005,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb0,
        name_start: 18016,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb1,
        name_start: 18027,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb2,
        name_start: 18038,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb3,
        name_start: 18048,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb4,
        name_start: 18058,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb5,
        name_start: 18069,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb6,
        name_start: 18080,
        name_len: 14,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb7,
        name_start: 18094,
        name_len: 14,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb8,
        name_start: 18108,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb9,
        name_start: 18117,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eba,
        name_start: 18126,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebb,
        name_start: 18131,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebc,
        name_start: 18136,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebd,
        name_start: 18142,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebe,
        name_start: 18148,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebf,
        name_start: 18164,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec0,
        name_start: 18180,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec1,
        name_start: 18196,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec2,
        name_start: 18212,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec3,
        name_start: 18227,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec4,
        name_start: 18242,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec5,
        name_start: 18258,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec6,
        name_start: 18274,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec7,
        name_start: 18293,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec8,
        name_start: 18312,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec9,
        name_start: 18317,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eca,
        name_start: 18322,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecb,
        name_start: 18331,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecc,
        name_start: 18340,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecd,
        name_start: 18349,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ece,
        name_start: 18358,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecf,
        name_start: 18363,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed0,
        name_start: 18368,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed1,
        name_start: 18384,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed2,
        name_start: 18400,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed3,
        name_start: 18416,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed4,
        name_start: 18432,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed5,
        name_start: 18447,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed6,
        name_start: 18462,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed7,
        name_start: 18478,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed8,
        name_start: 18494,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed9,
        name_start: 18513,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eda,
        name_start: 18532,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edb,
        name_start: 18542,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edc,
        name_start: 18552,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edd,
        name_start: 18562,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ede,
        name_start: 18572,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edf,
        name_start: 18581,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee0,
        name_start: 18590,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee1,
        name_start: 18600,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee2,
        name_start: 18610,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee3,
        name_start: 18623,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee4,
        name_start: 18636,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee5,
        name_start: 18645,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee6,
        name_start: 18654,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee7,
        name_start: 18659,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee8,
        name_start: 18664,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee9,
        name_start: 18674,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eea,
        name_start: 18684,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eeb,
        name_start: 18694,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eec,
        name_start: 18704,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eed,
        name_start: 18713,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eee,
        name_start: 18722,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eef,
        name_start: 18732,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef0,
        name_start: 18742,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef1,
        name_start: 18755,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef2,
        name_start: 18768,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef3,
        name_start: 18774,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef4,
        name_start: 18780,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef5,
        name_start: 18789,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef6,
        name_start: 18798,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef7,
        name_start: 18803,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef8,
        name_start: 18808,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef9,
        name_start: 18814,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002039,
        name_start: 18820,
        name_len: 24,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100203a,
        name_start: 18844,
        name_len: 25,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002070,
        name_start: 18869,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002074,
        name_start: 18881,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002075,
        name_start: 18893,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002076,
        name_start: 18905,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002077,
        name_start: 18916,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002078,
        name_start: 18929,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002079,
        name_start: 18942,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002080,
        name_start: 18954,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002081,
        name_start: 18967,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002082,
        name_start: 18979,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002083,
        name_start: 18991,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002084,
        name_start: 19005,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002085,
        name_start: 19018,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002086,
        name_start: 19031,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002087,
        name_start: 19043,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002088,
        name_start: 19057,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002089,
        name_start: 19071,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a0,
        name_start: 19084,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a1,
        name_start: 19091,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a2,
        name_start: 19100,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a3,
        name_start: 19112,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a4,
        name_start: 19122,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a5,
        name_start: 19130,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a6,
        name_start: 19138,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a7,
        name_start: 19147,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a8,
        name_start: 19157,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a9,
        name_start: 19166,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020aa,
        name_start: 19173,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020ab,
        name_start: 19186,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002202,
        name_start: 19194,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002205,
        name_start: 19210,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002208,
        name_start: 19218,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002209,
        name_start: 19227,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100220b,
        name_start: 19239,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100221a,
        name_start: 19249,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100221b,
        name_start: 19259,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100221c,
        name_start: 19267,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100222c,
        name_start: 19277,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100222d,
        name_start: 19286,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002235,
        name_start: 19295,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002247,
        name_start: 19302,
        name_len: 11,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002248,
        name_start: 19313,
        name_len: 8,
        flags: 0 | HAS_CHAR | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002262,
        name_start: 19321,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002263,
        name_start: 19333,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002800,
        name_start: 19341,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002801,
        name_start: 19354,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002802,
        name_start: 19368,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002803,
        name_start: 19382,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002804,
        name_start: 19397,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002805,
        name_start: 19411,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002806,
        name_start: 19426,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002807,
        name_start: 19441,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002808,
        name_start: 19457,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002809,
        name_start: 19471,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280a,
        name_start: 19486,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280b,
        name_start: 19501,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280c,
        name_start: 19517,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280d,
        name_start: 19532,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280e,
        name_start: 19548,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280f,
        name_start: 19564,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002810,
        name_start: 19581,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002811,
        name_start: 19595,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002812,
        name_start: 19610,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002813,
        name_start: 19625,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002814,
        name_start: 19641,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002815,
        name_start: 19656,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002816,
        name_start: 19672,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002817,
        name_start: 19688,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002818,
        name_start: 19705,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002819,
        name_start: 19720,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281a,
        name_start: 19736,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281b,
        name_start: 19752,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281c,
        name_start: 19769,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281d,
        name_start: 19785,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281e,
        name_start: 19802,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281f,
        name_start: 19819,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002820,
        name_start: 19837,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002821,
        name_start: 19851,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002822,
        name_start: 19866,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002823,
        name_start: 19881,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002824,
        name_start: 19897,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002825,
        name_start: 19912,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002826,
        name_start: 19928,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002827,
        name_start: 19944,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002828,
        name_start: 19961,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002829,
        name_start: 19976,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282a,
        name_start: 19992,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282b,
        name_start: 20008,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282c,
        name_start: 20025,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282d,
        name_start: 20041,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282e,
        name_start: 20058,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282f,
        name_start: 20075,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002830,
        name_start: 20093,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002831,
        name_start: 20108,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002832,
        name_start: 20124,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002833,
        name_start: 20140,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002834,
        name_start: 20157,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002835,
        name_start: 20173,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002836,
        name_start: 20190,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002837,
        name_start: 20207,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002838,
        name_start: 20225,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002839,
        name_start: 20241,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283a,
        name_start: 20258,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283b,
        name_start: 20275,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283c,
        name_start: 20293,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283d,
        name_start: 20310,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283e,
        name_start: 20328,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283f,
        name_start: 20346,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002840,
        name_start: 20365,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002841,
        name_start: 20379,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002842,
        name_start: 20394,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002843,
        name_start: 20409,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002844,
        name_start: 20425,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002845,
        name_start: 20440,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002846,
        name_start: 20456,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002847,
        name_start: 20472,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002848,
        name_start: 20489,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002849,
        name_start: 20504,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284a,
        name_start: 20520,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284b,
        name_start: 20536,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284c,
        name_start: 20553,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284d,
        name_start: 20569,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284e,
        name_start: 20586,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284f,
        name_start: 20603,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002850,
        name_start: 20621,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002851,
        name_start: 20636,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002852,
        name_start: 20652,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002853,
        name_start: 20668,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002854,
        name_start: 20685,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002855,
        name_start: 20701,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002856,
        name_start: 20718,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002857,
        name_start: 20735,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002858,
        name_start: 20753,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002859,
        name_start: 20769,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285a,
        name_start: 20786,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285b,
        name_start: 20803,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285c,
        name_start: 20821,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285d,
        name_start: 20838,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285e,
        name_start: 20856,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285f,
        name_start: 20874,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002860,
        name_start: 20893,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002861,
        name_start: 20908,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002862,
        name_start: 20924,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002863,
        name_start: 20940,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002864,
        name_start: 20957,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002865,
        name_start: 20973,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002866,
        name_start: 20990,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002867,
        name_start: 21007,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002868,
        name_start: 21025,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002869,
        name_start: 21041,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286a,
        name_start: 21058,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286b,
        name_start: 21075,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286c,
        name_start: 21093,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286d,
        name_start: 21110,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286e,
        name_start: 21128,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286f,
        name_start: 21146,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002870,
        name_start: 21165,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002871,
        name_start: 21181,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002872,
        name_start: 21198,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002873,
        name_start: 21215,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002874,
        name_start: 21233,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002875,
        name_start: 21250,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002876,
        name_start: 21268,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002877,
        name_start: 21286,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002878,
        name_start: 21305,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002879,
        name_start: 21322,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287a,
        name_start: 21340,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287b,
        name_start: 21358,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287c,
        name_start: 21377,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287d,
        name_start: 21395,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287e,
        name_start: 21414,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287f,
        name_start: 21433,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002880,
        name_start: 21453,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002881,
        name_start: 21467,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002882,
        name_start: 21482,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002883,
        name_start: 21497,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002884,
        name_start: 21513,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002885,
        name_start: 21528,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002886,
        name_start: 21544,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002887,
        name_start: 21560,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002888,
        name_start: 21577,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002889,
        name_start: 21592,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288a,
        name_start: 21608,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288b,
        name_start: 21624,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288c,
        name_start: 21641,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288d,
        name_start: 21657,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288e,
        name_start: 21674,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288f,
        name_start: 21691,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002890,
        name_start: 21709,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002891,
        name_start: 21724,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002892,
        name_start: 21740,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002893,
        name_start: 21756,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002894,
        name_start: 21773,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002895,
        name_start: 21789,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002896,
        name_start: 21806,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002897,
        name_start: 21823,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002898,
        name_start: 21841,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002899,
        name_start: 21857,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289a,
        name_start: 21874,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289b,
        name_start: 21891,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289c,
        name_start: 21909,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289d,
        name_start: 21926,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289e,
        name_start: 21944,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289f,
        name_start: 21962,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a0,
        name_start: 21981,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a1,
        name_start: 21996,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a2,
        name_start: 22012,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a3,
        name_start: 22028,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a4,
        name_start: 22045,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a5,
        name_start: 22061,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a6,
        name_start: 22078,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a7,
        name_start: 22095,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a8,
        name_start: 22113,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a9,
        name_start: 22129,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028aa,
        name_start: 22146,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ab,
        name_start: 22163,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ac,
        name_start: 22181,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ad,
        name_start: 22198,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ae,
        name_start: 22216,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028af,
        name_start: 22234,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b0,
        name_start: 22253,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b1,
        name_start: 22269,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b2,
        name_start: 22286,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b3,
        name_start: 22303,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b4,
        name_start: 22321,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b5,
        name_start: 22338,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b6,
        name_start: 22356,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b7,
        name_start: 22374,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b8,
        name_start: 22393,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b9,
        name_start: 22410,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ba,
        name_start: 22428,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bb,
        name_start: 22446,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bc,
        name_start: 22465,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bd,
        name_start: 22483,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028be,
        name_start: 22502,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bf,
        name_start: 22521,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c0,
        name_start: 22541,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c1,
        name_start: 22556,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c2,
        name_start: 22572,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c3,
        name_start: 22588,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c4,
        name_start: 22605,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c5,
        name_start: 22621,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c6,
        name_start: 22638,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c7,
        name_start: 22655,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c8,
        name_start: 22673,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c9,
        name_start: 22689,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ca,
        name_start: 22706,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cb,
        name_start: 22723,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cc,
        name_start: 22741,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cd,
        name_start: 22758,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ce,
        name_start: 22776,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cf,
        name_start: 22794,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d0,
        name_start: 22813,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d1,
        name_start: 22829,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d2,
        name_start: 22846,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d3,
        name_start: 22863,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d4,
        name_start: 22881,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d5,
        name_start: 22898,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d6,
        name_start: 22916,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d7,
        name_start: 22934,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d8,
        name_start: 22953,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d9,
        name_start: 22970,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028da,
        name_start: 22988,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028db,
        name_start: 23006,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028dc,
        name_start: 23025,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028dd,
        name_start: 23043,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028de,
        name_start: 23062,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028df,
        name_start: 23081,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e0,
        name_start: 23101,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e1,
        name_start: 23117,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e2,
        name_start: 23134,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e3,
        name_start: 23151,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e4,
        name_start: 23169,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e5,
        name_start: 23186,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e6,
        name_start: 23204,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e7,
        name_start: 23222,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e8,
        name_start: 23241,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e9,
        name_start: 23258,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ea,
        name_start: 23276,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028eb,
        name_start: 23294,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ec,
        name_start: 23313,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ed,
        name_start: 23331,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ee,
        name_start: 23350,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ef,
        name_start: 23369,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f0,
        name_start: 23389,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f1,
        name_start: 23406,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f2,
        name_start: 23424,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f3,
        name_start: 23442,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f4,
        name_start: 23461,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f5,
        name_start: 23479,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f6,
        name_start: 23498,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f7,
        name_start: 23517,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f8,
        name_start: 23537,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f9,
        name_start: 23555,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fa,
        name_start: 23574,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fb,
        name_start: 23593,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fc,
        name_start: 23613,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fd,
        name_start: 23632,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fe,
        name_start: 23652,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ff,
        name_start: 23672,
        name_len: 21,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000a8,
        name_start: 23693,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000084c,
        name_start: 23705,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000a9,
        name_start: 23715,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000084e,
        name_start: 23727,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000aa,
        name_start: 23737,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000850,
        name_start: 23755,
        name_len: 16,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000ab,
        name_start: 23771,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000852,
        name_start: 23787,
        name_len: 14,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000ac,
        name_start: 23801,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000854,
        name_start: 23818,
        name_len: 15,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000af,
        name_start: 23833,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000856,
        name_start: 23839,
        name_len: 4,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000be,
        name_start: 23843,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000858,
        name_start: 23852,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000ee,
        name_start: 23859,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000085a,
        name_start: 23871,
        name_len: 4,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000085a,
        name_start: 23875,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000f6,
        name_start: 23877,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000085d,
        name_start: 23888,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000fc,
        name_start: 23897,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000085f,
        name_start: 23904,
        name_len: 5,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe22,
        name_start: 23909,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe27,
        name_start: 23919,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe2c,
        name_start: 23932,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe5e,
        name_start: 23947,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe60,
        name_start: 23965,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe7e,
        name_start: 23978,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000feb0,
        name_start: 23984,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff00,
        name_start: 23996,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff48,
        name_start: 24003,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff49,
        name_start: 24014,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6c,
        name_start: 24025,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000086b,
        name_start: 24032,
        name_len: 5,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6d,
        name_start: 24037,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000086d,
        name_start: 24045,
        name_len: 6,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6e,
        name_start: 24051,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000086f,
        name_start: 24057,
        name_len: 4,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6f,
        name_start: 24061,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000871,
        name_start: 24072,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff70,
        name_start: 24081,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000873,
        name_start: 24093,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff71,
        name_start: 24103,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000875,
        name_start: 24115,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff72,
        name_start: 24125,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000877,
        name_start: 24137,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff73,
        name_start: 24147,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000879,
        name_start: 24159,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff74,
        name_start: 24169,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000087b,
        name_start: 24178,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff75,
        name_start: 24185,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000087d,
        name_start: 24197,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff76,
        name_start: 24207,
        name_len: 10,
        flags: 0 | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff77,
        name_start: 24217,
        name_len: 10,
        flags: 0 | IS_DEPRECATED,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff02,
        name_start: 24227,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff03,
        name_start: 24234,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff04,
        name_start: 24240,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff07,
        name_start: 24248,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff08,
        name_start: 24258,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff0b,
        name_start: 24270,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff1b,
        name_start: 24278,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff31,
        name_start: 24287,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff32,
        name_start: 24297,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff33,
        name_start: 24312,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff40,
        name_start: 24325,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff41,
        name_start: 24336,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff42,
        name_start: 24345,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff43,
        name_start: 24356,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff44,
        name_start: 24368,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff45,
        name_start: 24379,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff51,
        name_start: 24389,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff52,
        name_start: 24396,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff53,
        name_start: 24401,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff54,
        name_start: 24409,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff57,
        name_start: 24416,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff58,
        name_start: 24426,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff59,
        name_start: 24438,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5a,
        name_start: 24448,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5b,
        name_start: 24460,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5c,
        name_start: 24471,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5d,
        name_start: 24482,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5e,
        name_start: 24494,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff60,
        name_start: 24506,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff63,
        name_start: 24515,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff65,
        name_start: 24524,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff67,
        name_start: 24531,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff69,
        name_start: 24538,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff6a,
        name_start: 24547,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff71,
        name_start: 24554,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff72,
        name_start: 24566,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff73,
        name_start: 24580,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff74,
        name_start: 24591,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff78,
        name_start: 24600,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ffff,
        name_start: 24610,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff00,
        name_start: 24619,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff01,
        name_start: 24630,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff02,
        name_start: 24642,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff03,
        name_start: 24653,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff04,
        name_start: 24664,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff05,
        name_start: 24679,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff10,
        name_start: 24692,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff11,
        name_start: 24698,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff60,
        name_start: 24704,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff70,
        name_start: 24714,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff71,
        name_start: 24722,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff72,
        name_start: 24730,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff73,
        name_start: 24737,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff74,
        name_start: 24744,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff75,
        name_start: 24752,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff76,
        name_start: 24758,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff77,
        name_start: 24772,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff78,
        name_start: 24791,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff79,
        name_start: 24803,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7a,
        name_start: 24822,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7b,
        name_start: 24837,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7c,
        name_start: 24860,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7d,
        name_start: 24883,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810a4,
        name_start: 24902,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810ae,
        name_start: 24920,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810d1,
        name_start: 24928,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810dc,
        name_start: 24946,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810f4,
        name_start: 24955,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c4,
        name_start: 24973,
        name_len: 21,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810f5,
        name_start: 24994,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081160,
        name_start: 25008,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081162,
        name_start: 25014,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081166,
        name_start: 25022,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081168,
        name_start: 25030,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008116a,
        name_start: 25044,
        name_len: 27,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008116e,
        name_start: 25071,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081170,
        name_start: 25090,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081171,
        name_start: 25111,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081175,
        name_start: 25129,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081177,
        name_start: 25149,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081178,
        name_start: 25164,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081179,
        name_start: 25181,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008117a,
        name_start: 25198,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008117b,
        name_start: 25218,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008117c,
        name_start: 25236,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008117d,
        name_start: 25258,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081180,
        name_start: 25282,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081181,
        name_start: 25301,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081182,
        name_start: 25321,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081183,
        name_start: 25341,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081184,
        name_start: 25356,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081185,
        name_start: 25379,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008dc,
        name_start: 25386,
        name_len: 18,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081186,
        name_start: 25404,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081188,
        name_start: 25428,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081192,
        name_start: 25437,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081193,
        name_start: 25450,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081199,
        name_start: 25465,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008119b,
        name_start: 25482,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008119d,
        name_start: 25491,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a0,
        name_start: 25510,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a4,
        name_start: 25524,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a6,
        name_start: 25537,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a8,
        name_start: 25547,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a9,
        name_start: 25565,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811aa,
        name_start: 25581,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811ac,
        name_start: 25593,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811ad,
        name_start: 25606,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811af,
        name_start: 25621,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b0,
        name_start: 25638,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b6,
        name_start: 25652,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b7,
        name_start: 25667,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b8,
        name_start: 25682,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b9,
        name_start: 25698,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811ba,
        name_start: 25716,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811bc,
        name_start: 25726,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811bd,
        name_start: 25748,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811be,
        name_start: 25763,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811bf,
        name_start: 25778,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d0,
        name_start: 25791,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d1,
        name_start: 25797,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d2,
        name_start: 25807,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d3,
        name_start: 25816,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d4,
        name_start: 25825,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d5,
        name_start: 25834,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d6,
        name_start: 25843,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d7,
        name_start: 25852,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d8,
        name_start: 25861,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d9,
        name_start: 25870,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811da,
        name_start: 25879,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811db,
        name_start: 25888,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811dc,
        name_start: 25898,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811dd,
        name_start: 25908,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811de,
        name_start: 25918,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811df,
        name_start: 25926,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e0,
        name_start: 25934,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e1,
        name_start: 25942,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e2,
        name_start: 25950,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e3,
        name_start: 25958,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e4,
        name_start: 25966,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e5,
        name_start: 25974,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081200,
        name_start: 25990,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081201,
        name_start: 26002,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081202,
        name_start: 26014,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081203,
        name_start: 26026,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081204,
        name_start: 26038,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081205,
        name_start: 26050,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081206,
        name_start: 26062,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081207,
        name_start: 26074,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081208,
        name_start: 26086,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081209,
        name_start: 26098,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120a,
        name_start: 26110,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120b,
        name_start: 26125,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120c,
        name_start: 26141,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120d,
        name_start: 26153,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120e,
        name_start: 26165,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120f,
        name_start: 26177,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081210,
        name_start: 26189,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081211,
        name_start: 26204,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081215,
        name_start: 26217,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081216,
        name_start: 26233,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081217,
        name_start: 26250,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081218,
        name_start: 26262,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081219,
        name_start: 26276,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121a,
        name_start: 26290,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121b,
        name_start: 26305,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121c,
        name_start: 26320,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121d,
        name_start: 26336,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121e,
        name_start: 26355,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081230,
        name_start: 26371,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081232,
        name_start: 26384,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081240,
        name_start: 26405,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081241,
        name_start: 26421,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081242,
        name_start: 26436,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081243,
        name_start: 26447,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081244,
        name_start: 26463,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081245,
        name_start: 26476,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081246,
        name_start: 26491,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081247,
        name_start: 26507,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081249,
        name_start: 26520,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124a,
        name_start: 26535,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124b,
        name_start: 26546,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124c,
        name_start: 26568,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124d,
        name_start: 26591,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124e,
        name_start: 26613,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124f,
        name_start: 26630,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081250,
        name_start: 26646,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081251,
        name_start: 26663,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081252,
        name_start: 26680,
        name_len: 29,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081253,
        name_start: 26709,
        name_len: 30,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081254,
        name_start: 26739,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081255,
        name_start: 26760,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081256,
        name_start: 26780,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081260,
        name_start: 26799,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081261,
        name_start: 26821,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081262,
        name_start: 26843,
        name_len: 27,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081263,
        name_start: 26870,
        name_len: 27,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081264,
        name_start: 26897,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081265,
        name_start: 26921,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081266,
        name_start: 26945,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081267,
        name_start: 26956,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081268,
        name_start: 26969,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081269,
        name_start: 26979,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126a,
        name_start: 26991,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126b,
        name_start: 27003,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126c,
        name_start: 27019,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126d,
        name_start: 27032,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126e,
        name_start: 27045,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126f,
        name_start: 27058,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081270,
        name_start: 27068,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081271,
        name_start: 27084,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081272,
        name_start: 27098,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081273,
        name_start: 27113,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081274,
        name_start: 27120,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081275,
        name_start: 27130,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081276,
        name_start: 27145,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081277,
        name_start: 27160,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081278,
        name_start: 27168,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081279,
        name_start: 27188,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127a,
        name_start: 27211,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127b,
        name_start: 27234,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127c,
        name_start: 27249,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127d,
        name_start: 27268,
        name_len: 25,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127e,
        name_start: 27293,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127f,
        name_start: 27309,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081280,
        name_start: 27316,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081281,
        name_start: 27328,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081282,
        name_start: 27344,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081283,
        name_start: 27364,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081284,
        name_start: 27382,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081285,
        name_start: 27398,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081286,
        name_start: 27418,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081287,
        name_start: 27434,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081288,
        name_start: 27449,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081290,
        name_start: 27460,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081291,
        name_start: 27470,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081292,
        name_start: 27480,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081293,
        name_start: 27490,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081294,
        name_start: 27500,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081295,
        name_start: 27510,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081296,
        name_start: 27520,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081297,
        name_start: 27530,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081298,
        name_start: 27540,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081299,
        name_start: 27550,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129a,
        name_start: 27561,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129b,
        name_start: 27572,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129c,
        name_start: 27583,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129d,
        name_start: 27594,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129e,
        name_start: 27605,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129f,
        name_start: 27616,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a0,
        name_start: 27627,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a1,
        name_start: 27638,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a2,
        name_start: 27649,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a3,
        name_start: 27660,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a4,
        name_start: 27671,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a5,
        name_start: 27682,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a6,
        name_start: 27693,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a7,
        name_start: 27704,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a8,
        name_start: 27715,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a9,
        name_start: 27726,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812aa,
        name_start: 27737,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ab,
        name_start: 27748,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ac,
        name_start: 27759,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ad,
        name_start: 27770,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b0,
        name_start: 27781,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b1,
        name_start: 27801,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b2,
        name_start: 27820,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b3,
        name_start: 27840,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b4,
        name_start: 27856,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b5,
        name_start: 27872,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b8,
        name_start: 27888,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b9,
        name_start: 27903,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ba,
        name_start: 27918,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812bb,
        name_start: 27933,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812bc,
        name_start: 27948,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812bd,
        name_start: 27963,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe01,
        name_start: 27982,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe02,
        name_start: 27997,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe03,
        name_start: 28012,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe04,
        name_start: 28027,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe05,
        name_start: 28042,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe06,
        name_start: 28057,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe07,
        name_start: 28072,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe08,
        name_start: 28087,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe09,
        name_start: 28102,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe0a,
        name_start: 28117,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe0b,
        name_start: 28133,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe0c,
        name_start: 28149,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe20,
        name_start: 28165,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe21,
        name_start: 28175,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe22,
        name_start: 28188,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe23,
        name_start: 28202,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe24,
        name_start: 28216,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe25,
        name_start: 28233,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff01,
        name_start: 28248,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff02,
        name_start: 28260,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff03,
        name_start: 28279,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff04,
        name_start: 28300,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff05,
        name_start: 28317,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff06,
        name_start: 28336,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff07,
        name_start: 28357,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff10,
        name_start: 28379,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff11,
        name_start: 28390,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff12,
        name_start: 28410,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff13,
        name_start: 28423,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff14,
        name_start: 28443,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff15,
        name_start: 28456,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff16,
        name_start: 28469,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff17,
        name_start: 28482,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff18,
        name_start: 28495,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff19,
        name_start: 28507,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1a,
        name_start: 28515,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1b,
        name_start: 28524,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1c,
        name_start: 28534,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1d,
        name_start: 28549,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1e,
        name_start: 28563,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1f,
        name_start: 28571,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff20,
        name_start: 28583,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff21,
        name_start: 28595,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff22,
        name_start: 28608,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff23,
        name_start: 28626,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff24,
        name_start: 28638,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff25,
        name_start: 28652,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff26,
        name_start: 28667,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff27,
        name_start: 28675,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff28,
        name_start: 28686,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff29,
        name_start: 28694,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2a,
        name_start: 28705,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2b,
        name_start: 28717,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2c,
        name_start: 28727,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2d,
        name_start: 28736,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2e,
        name_start: 28751,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2f,
        name_start: 28758,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff30,
        name_start: 28767,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff31,
        name_start: 28780,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff32,
        name_start: 28794,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff33,
        name_start: 28808,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff34,
        name_start: 28822,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff35,
        name_start: 28836,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff36,
        name_start: 28849,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff37,
        name_start: 28857,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff38,
        name_start: 28868,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff39,
        name_start: 28879,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3a,
        name_start: 28894,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3b,
        name_start: 28906,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3c,
        name_start: 28926,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3d,
        name_start: 28937,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3e,
        name_start: 28950,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3f,
        name_start: 28965,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff40,
        name_start: 28980,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff41,
        name_start: 28991,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff42,
        name_start: 29002,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff43,
        name_start: 29013,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff44,
        name_start: 29024,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff45,
        name_start: 29035,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff46,
        name_start: 29046,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff47,
        name_start: 29057,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff48,
        name_start: 29068,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff49,
        name_start: 29079,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4a,
        name_start: 29090,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4b,
        name_start: 29101,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4c,
        name_start: 29112,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4d,
        name_start: 29123,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4e,
        name_start: 29134,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4f,
        name_start: 29145,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff50,
        name_start: 29156,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff51,
        name_start: 29175,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff52,
        name_start: 29195,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff53,
        name_start: 29203,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f1,
        name_start: 29209,
        name_len: 17,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff54,
        name_start: 29226,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff55,
        name_start: 29240,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff56,
        name_start: 29249,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff57,
        name_start: 29258,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff58,
        name_start: 29266,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff59,
        name_start: 29273,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5a,
        name_start: 29284,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5b,
        name_start: 29291,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5c,
        name_start: 29304,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5d,
        name_start: 29313,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5e,
        name_start: 29325,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5f,
        name_start: 29333,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff60,
        name_start: 29339,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff61,
        name_start: 29349,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff62,
        name_start: 29359,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff63,
        name_start: 29369,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff65,
        name_start: 29380,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff66,
        name_start: 29390,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff67,
        name_start: 29400,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff68,
        name_start: 29411,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff69,
        name_start: 29418,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6a,
        name_start: 29426,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6b,
        name_start: 29440,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6c,
        name_start: 29448,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6d,
        name_start: 29458,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6e,
        name_start: 29467,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff70,
        name_start: 29476,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff72,
        name_start: 29481,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff73,
        name_start: 29490,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff74,
        name_start: 29500,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff75,
        name_start: 29517,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff76,
        name_start: 29531,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff77,
        name_start: 29545,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff78,
        name_start: 29553,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff79,
        name_start: 29565,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7a,
        name_start: 29579,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7b,
        name_start: 29594,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7c,
        name_start: 29602,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7d,
        name_start: 29611,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7e,
        name_start: 29626,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7f,
        name_start: 29637,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff80,
        name_start: 29649,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff81,
        name_start: 29661,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff82,
        name_start: 29670,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff84,
        name_start: 29680,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff85,
        name_start: 29690,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff86,
        name_start: 29701,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff87,
        name_start: 29712,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff88,
        name_start: 29721,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff89,
        name_start: 29736,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8a,
        name_start: 29744,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8b,
        name_start: 29752,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8c,
        name_start: 29762,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8d,
        name_start: 29773,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8e,
        name_start: 29781,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8f,
        name_start: 29794,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff90,
        name_start: 29804,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff91,
        name_start: 29819,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff92,
        name_start: 29831,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff93,
        name_start: 29840,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff94,
        name_start: 29851,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff95,
        name_start: 29864,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff96,
        name_start: 29872,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff97,
        name_start: 29879,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff98,
        name_start: 29895,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff99,
        name_start: 29910,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9a,
        name_start: 29929,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9b,
        name_start: 29941,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9c,
        name_start: 29960,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9d,
        name_start: 29974,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9e,
        name_start: 29987,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9f,
        name_start: 30003,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa0,
        name_start: 30011,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa1,
        name_start: 30021,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa2,
        name_start: 30029,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa3,
        name_start: 30040,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa4,
        name_start: 30047,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa5,
        name_start: 30056,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa6,
        name_start: 30066,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa7,
        name_start: 30074,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa8,
        name_start: 30085,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa9,
        name_start: 30098,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb0,
        name_start: 30116,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb1,
        name_start: 30130,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb2,
        name_start: 30145,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb3,
        name_start: 30161,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb4,
        name_start: 30173,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb5,
        name_start: 30181,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb6,
        name_start: 30191,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb7,
        name_start: 30206,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb8,
        name_start: 30228,
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
    /// dead_apostrophe
    pub const dead_apostrophe: Keysym = Keysym(0x0000fe8e);
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
    /// ISO_Group_Shift
    pub const ISO_Group_Shift: Keysym = Keysym(0x0000ff7e);
    /// Mode_switch
    pub const Mode_switch: Keysym = Keysym(0x0000ff7e);
    /// script_switch
    pub const script_switch: Keysym = Keysym(0x0000ff7e);
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
    /// SSHARP
    pub const SSHARP: Keysym = Keysym(0x01001e9e);
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
    /// leftsingleanglequotemark
    pub const leftsingleanglequotemark: Keysym = Keysym(0x01002039);
    /// rightsingleanglequotemark
    pub const rightsingleanglequotemark: Keysym = Keysym(0x0100203a);
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
    /// XF86ElectronicPrivacyScreenOn
    pub const XF86ElectronicPrivacyScreenOn: Keysym = Keysym(0x10081252);
    /// XF86ElectronicPrivacyScreenOff
    pub const XF86ElectronicPrivacyScreenOff: Keysym = Keysym(0x10081253);
    /// XF86ActionOnSelection
    pub const XF86ActionOnSelection: Keysym = Keysym(0x10081254);
    /// XF86ContextualInsert
    pub const XF86ContextualInsert: Keysym = Keysym(0x10081255);
    /// XF86ContextualQuery
    pub const XF86ContextualQuery: Keysym = Keysym(0x10081256);
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

