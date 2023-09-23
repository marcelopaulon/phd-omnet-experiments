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

network DADCA_ACK_n100_u4_a8000m2_4
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
        @display("i=misc/sensor;p=996.92,7987.93");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2107.79,7875.51");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3814.81,7967.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4246.32,7922.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4735.68,7975.45");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=434.27,7707.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=773.77,7575.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1260.70,7740.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1621.11,7579.25");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2682.98,7616.43");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3108.48,7743.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4923.75,7555.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5369.04,7728.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1982.47,7391.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3116.64,7334.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3905.52,7444.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5842.99,7356.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=421.14,7199.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1057.63,7206.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1598.76,7225.64");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2285.07,7210.60");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3485.41,7158.66");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4444.03,7204.27");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1620.04,6875.54");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2243.03,6830.35");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2681.00,6945.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3936.30,6817.93");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4601.52,6872.25");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5382.67,6959.70");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5792.65,6931.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=314.83,6624.60");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=732.22,6590.23");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1248.30,6720.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3416.24,6699.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5290.66,6571.95");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5707.39,6545.31");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1726.12,6512.34");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3194.09,6293.18");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3557.77,6352.05");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4274.96,6508.06");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4590.43,6315.93");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4984.37,6399.99");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=291.50,6129.67");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=860.52,6212.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1358.52,6256.14");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2308.50,6130.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3914.81,6220.64");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1408.27,5817.36");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1920.07,5834.36");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2825.79,6010.38");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3688.12,5905.78");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4207.39,6001.74");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4741.36,5854.63");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5295.37,5999.17");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5708.25,5916.27");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=534.75,5752.54");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=912.68,5555.66");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2471.37,5622.07");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3339.08,5697.04");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4089.65,5658.61");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=133.29,5331.62");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1475.00,5355.62");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1965.41,5339.42");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3567.46,5428.15");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3986.04,5305.58");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4488.71,5413.84");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5148.54,5413.51");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5538.12,5439.46");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=411.66,5112.16");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=750.12,5220.36");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1122.40,5247.14");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1714.36,5081.90");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2974.03,5192.64");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4777.33,5202.34");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=16.88,4867.66");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1293.39,4934.72");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2089.46,4883.14");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2613.80,5022.79");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3561.35,4893.99");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4213.61,4793.75");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5308.74,4942.57");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5974.10,4937.63");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=573.93,4621.13");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1687.26,4659.95");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2665.29,4566.04");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3127.94,4778.82");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4629.85,4737.68");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5632.48,4566.36");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=91.38,4343.75");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=902.95,4397.08");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1249.76,4311.37");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2334.40,4443.58");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3626.08,4519.81");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4264.58,4440.13");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5040.38,4524.35");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=397.12,4076.40");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1694.55,4051.46");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3991.58,4201.86");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4684.14,4171.94");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5155.10,4177.21");
    }

}

