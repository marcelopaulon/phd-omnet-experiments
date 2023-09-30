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

network DADCA_ACK_n100_u3_a8000m2_4
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
        @display("i=misc/sensor;p=829.59,7941.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2145.36,7930.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3495.67,7937.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3918.00,7883.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4837.94,7826.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5861.59,7969.50");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=150.03,7682.90");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=640.54,7525.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1146.70,7740.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1577.75,7644.71");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2644.81,7735.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3109.44,7733.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4233.96,7552.56");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5981.98,7538.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2149.41,7401.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2501.55,7387.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3355.81,7389.43");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3859.09,7400.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4470.71,7278.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4856.65,7291.98");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5330.94,7414.65");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=283.07,7255.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=933.52,7234.82");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1336.75,7180.36");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1943.64,7077.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5779.57,7015.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=538.13,6873.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1106.87,6907.54");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2726.05,6825.50");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3891.99,6860.05");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4408.91,6933.42");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5445.43,6810.02");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=132.46,6563.45");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1357.58,6518.53");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1748.54,6701.53");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2063.08,6524.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3098.25,6655.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4689.45,6518.11");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5036.94,6737.70");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=680.55,6453.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2423.84,6269.87");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2745.83,6418.89");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3868.03,6450.25");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4387.37,6339.05");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5323.35,6314.44");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5754.55,6391.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1015.93,6185.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1377.90,6061.38");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1839.43,6067.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3565.20,6250.58");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4714.75,6158.35");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5905.04,6039.68");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=230.99,5962.93");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=678.26,5867.70");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2229.33,5930.37");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2618.90,5868.76");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3152.54,6002.03");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3997.18,5920.76");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4582.67,5819.32");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5147.24,5878.06");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1602.98,5528.80");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3260.61,5597.33");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5707.00,5637.66");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=216.06,5463.70");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=833.66,5376.63");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1197.25,5453.25");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2165.86,5476.56");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2949.95,5391.85");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4382.56,5513.94");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4994.69,5427.60");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=463.57,5066.52");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1723.12,5057.21");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2591.74,5262.25");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3290.05,5230.72");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3676.69,5103.97");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4039.36,5097.57");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4534.57,5174.48");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5509.81,5257.74");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5949.45,5207.64");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=9.17,4844.86");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2189.28,4821.98");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2699.97,4842.39");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4982.63,4935.34");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=688.92,4604.14");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1276.10,4780.41");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1629.32,4594.33");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3022.91,4647.57");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4245.71,4730.46");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5603.83,4701.23");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=961.86,4353.74");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3628.69,4503.56");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4542.65,4288.22");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4798.22,4533.41");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5308.66,4464.32");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=109.19,4225.50");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1230.05,4105.90");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1796.37,4155.67");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2194.59,4286.59");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2582.69,4275.56");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2998.50,4168.92");
    }

}

