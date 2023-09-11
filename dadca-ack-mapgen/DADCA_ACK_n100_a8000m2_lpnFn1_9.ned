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

network DADCA_ACK_n100_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=138.77,168.35");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=202.72,199.33");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=211.87,261.47");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=130.41,245.45");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=291.19,200.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=291.57,137.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=180.54,114.27");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=262.06,304.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=252.39,402.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=269.59,494.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=184.11,375.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=159.27,464.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=321.10,279.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=359.15,341.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=186.10,553.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=178.92,630.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=116.86,626.91");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=249.59,571.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=331.31,524.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=410.84,517.55");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=391.81,456.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=480.23,443.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=492.78,354.99");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=542.23,403.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=625.83,431.93");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=614.19,493.66");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=586.94,565.38");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=659.70,535.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=737.69,541.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=824.19,524.53");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=906.55,550.78");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=935.04,483.90");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=980.16,404.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=1011.12,493.64");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=986.10,564.23");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=914.85,419.55");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=955.60,666.29");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=962.79,732.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=873.93,707.12");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=832.71,754.93");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=773.99,712.12");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=745.03,824.93");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=689.05,796.74");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=653.01,879.06");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=628.22,737.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=564.49,795.06");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=601.10,638.71");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=662.46,615.85");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=534.75,488.93");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=525.04,633.69");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=526.10,732.28");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=494.41,866.60");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=631.14,822.85");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=586.20,864.64");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=608.86,944.27");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=457.07,817.24");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=381.63,829.80");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=610.36,1028.21");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=535.19,1042.33");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=499.96,935.87");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=311.78,772.57");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=230.39,828.94");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=230.95,902.38");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=159.47,779.78");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=244.74,712.60");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=410.31,735.37");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=417.20,600.92");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=315.99,445.89");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=322.43,605.22");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=126.45,564.85");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=66.02,563.96");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=37.21,627.83");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=299.86,847.19");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=314.15,913.01");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=402.35,940.15");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=456.07,977.06");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=569.80,330.03");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=470.68,510.04");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=431.15,368.07");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=375.67,220.57");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=453.34,168.57");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=508.07,213.87");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=469.46,262.93");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=870.03,609.17");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1011.44,636.53");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=783.05,585.12");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=808.31,642.90");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=716.90,651.90");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1093.85,652.52");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1095.72,721.08");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1162.03,723.59");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1216.55,654.44");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1265.70,715.33");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1337.10,708.43");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1430.18,686.77");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1397.46,590.71");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1345.86,632.53");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1409.20,768.70");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1518.26,686.91");
    }

}

