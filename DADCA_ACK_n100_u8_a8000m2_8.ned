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

network DADCA_ACK_n100_u8_a8000m2_8
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
        @display("i=misc/sensor;p=1304.04,7980.72");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1665.93,7911.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2168.35,7767.07");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3045.74,7865.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3813.90,7966.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4777.51,7795.79");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5389.78,7931.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=406.89,7590.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1119.32,7670.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1574.29,7526.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2525.23,7625.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5389.61,7568.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=120.42,7389.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1843.90,7288.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3042.09,7362.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3577.24,7372.86");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4089.80,7435.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=925.13,7132.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1496.75,7083.62");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2478.10,7232.20");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3894.83,7065.70");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4286.00,7110.16");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4678.23,7183.08");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5809.85,7061.30");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=500.30,6878.44");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2247.92,6909.74");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2688.58,6773.90");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3126.50,6984.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5053.39,6999.53");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5425.69,6989.14");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=49.87,6720.58");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2115.48,6579.39");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3527.88,6720.18");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3973.74,6612.39");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4347.73,6696.11");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4807.35,6687.35");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=398.27,6280.20");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=890.13,6328.23");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2684.13,6310.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3071.51,6425.67");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5875.11,6391.43");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=79.72,6058.61");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1480.43,6119.95");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1930.14,6191.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3471.17,6237.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4413.59,6151.93");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4893.64,6102.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5623.36,6113.24");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=946.42,5947.68");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2289.78,5948.62");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3124.50,5951.03");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3964.89,5953.64");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5255.82,5861.67");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5900.11,5865.54");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=73.35,5525.27");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1504.05,5708.62");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3278.77,5541.65");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4372.52,5726.00");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4754.54,5582.48");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=466.53,5475.64");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1225.02,5358.08");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1915.97,5378.37");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2801.67,5468.04");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4117.80,5373.01");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5327.44,5303.25");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=656.30,5136.14");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2382.93,5076.65");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3918.28,5072.47");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4371.22,5068.20");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4705.88,5232.34");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5866.38,5273.25");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=256.41,5023.16");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1083.29,4973.42");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2137.63,4810.22");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2759.02,5025.87");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3122.96,4938.03");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4633.59,4835.36");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5176.29,4907.27");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5959.28,4919.97");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1756.78,4778.04");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2901.67,4626.92");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3791.16,4718.75");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4166.53,4653.86");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5604.11,4727.61");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1.64,4531.82");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=443.41,4475.03");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1359.28,4494.49");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2437.65,4509.45");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3350.33,4443.55");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3796.41,4298.10");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4408.17,4306.32");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4844.32,4412.12");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5345.65,4486.25");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5792.36,4332.67");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=962.60,4194.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2114.62,4192.51");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2722.44,4241.72");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3080.57,4207.96");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5377.33,4110.95");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=131.33,3861.91");
    }

}

