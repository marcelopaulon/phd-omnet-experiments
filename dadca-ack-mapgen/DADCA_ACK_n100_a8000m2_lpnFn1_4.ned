package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_a8000m2_lpnFn1_4
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=20.35,61.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=29.12,187.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=91.76,189.36");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=34.26,282.90");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=172.04,241.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=215.22,318.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=185.66,397.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=263.51,403.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=278.98,308.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=39.29,377.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=364.88,304.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=396.92,239.72");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=432.31,330.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=458.82,396.93");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=309.03,148.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=252.63,173.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=127.72,297.41");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=103.35,385.99");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=104.29,461.15");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=155.43,508.82");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=207.00,543.20");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=282.73,546.08");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=339.07,626.60");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=400.89,615.34");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=441.24,683.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=434.40,747.93");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=427.51,529.49");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=367.08,546.12");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=274.08,611.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=318.04,684.53");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=260.62,720.52");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=145.13,593.42");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=202.88,113.72");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=222.58,2.06");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=177.53,181.84");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=302.92,49.13");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=353.76,15.90");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=429.82,3.31");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=400.71,64.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=462.89,126.92");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=468.00,260.15");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=493.33,525.45");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=248.35,480.49");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=326.81,365.57");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=234.06,233.32");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=262.89,101.36");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=167.83,33.83");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=19.90,460.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=99.99,0.04");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=26.35,124.71");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=209.33,606.03");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=349.20,483.95");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=407.76,462.42");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=509.49,583.72");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=461.08,621.29");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=592.01,562.11");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=633.28,498.48");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=712.61,505.03");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=722.41,576.74");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=719.99,667.16");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=829.29,535.42");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=825.75,452.86");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=906.12,489.60");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=901.21,564.79");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=868.77,663.04");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=837.51,598.78");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=630.09,693.00");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=626.55,780.96");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=568.89,718.30");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=706.63,426.00");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=939.25,648.39");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=805.52,762.75");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=738.22,778.14");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=697.22,873.51");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=735.95,939.81");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=763.03,1005.16");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=760.44,1094.97");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=811.03,1169.85");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=698.64,1064.16");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=672.17,960.32");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=606.88,888.97");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=552.89,853.19");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=483.73,848.87");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=466.15,931.02");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=587.92,1014.06");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=587.68,1106.65");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=681.05,1129.94");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=826.46,1110.82");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=775.66,1262.69");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=888.03,1057.10");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=978.14,1059.82");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1053.56,1095.72");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1001.39,1156.27");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=924.11,1107.65");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=971.59,997.41");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1056.12,1005.67");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1092.25,1188.29");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=875.77,997.77");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=636.87,1192.00");
    }

}

