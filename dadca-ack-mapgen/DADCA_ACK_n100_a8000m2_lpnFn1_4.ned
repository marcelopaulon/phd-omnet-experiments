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
        @display("i=misc/sensor,p=4.99,78.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=141.93,151.00");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=102.51,285.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=50.09,234.23");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=191.58,251.79");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=239.31,214.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=272.95,267.55");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=341.52,260.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=290.04,334.91");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=219.55,358.23");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=226.91,132.35");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=27.01,171.37");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=350.59,362.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=412.57,449.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=466.85,406.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=322.22,514.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=246.49,532.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=265.27,454.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=324.63,574.84");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=391.06,521.33");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=405.24,584.15");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=374.56,643.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=460.07,634.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=501.65,726.13");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=559.66,636.71");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=570.16,545.87");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=594.91,462.54");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=538.64,386.71");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=484.92,468.57");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=169.75,576.78");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=95.73,561.37");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=82.80,491.94");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=140.54,454.69");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=228.39,602.92");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=141.51,515.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=114.75,392.47");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=56.00,365.39");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=17.04,483.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=164.48,649.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=115.25,696.09");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=167.62,762.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=79.49,797.63");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=123.95,873.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=211.80,844.97");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=272.14,913.64");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=322.07,853.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=343.77,788.72");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=276.23,785.20");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=294.68,673.14");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=232.77,714.82");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=420.45,764.62");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=406.08,699.99");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=396.60,825.15");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=474.50,810.89");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=653.91,573.88");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=705.38,652.50");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=577.78,321.77");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=514.13,301.20");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=435.82,316.09");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=331.22,187.17");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=70.85,17.23");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=107.51,203.20");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=167.79,61.65");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=262.05,33.56");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=339.40,431.79");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=412.15,895.86");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=365.71,940.62");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=323.56,727.57");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=493.89,546.66");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=616.75,401.38");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=631.44,267.61");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=590.26,202.56");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=592.09,118.34");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=505.25,78.80");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=443.98,98.31");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=390.59,24.49");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=327.52,121.14");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=321.86,15.61");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=184.29,412.35");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=744.89,528.66");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=692.89,484.13");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=768.11,428.29");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=724.43,368.84");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=686.97,302.85");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=806.75,379.43");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=890.24,358.64");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=972.45,383.04");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1056.27,409.72");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1112.17,454.20");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1079.72,515.99");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1148.51,505.83");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1215.57,515.14");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1220.26,575.77");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1357.36,494.62");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1343.85,556.82");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1382.23,641.30");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1325.90,704.08");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1250.97,685.81");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1455.99,680.64");
    }

}

