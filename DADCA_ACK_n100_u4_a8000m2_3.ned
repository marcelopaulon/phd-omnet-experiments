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

network DADCA_ACK_n100_u4_a8000m2_3
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
        @display("i=misc/sensor;p=1015.48,7874.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1502.26,7852.57");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1934.81,7833.79");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2979.43,7811.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4073.63,7789.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4498.35,7770.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4884.11,7853.41");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=529.11,7724.57");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1234.06,7593.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2515.82,7729.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3360.45,7730.23");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5355.85,7576.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5730.05,7665.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=862.68,7402.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1616.38,7463.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3128.29,7426.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4769.25,7399.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=707.58,7070.41");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1294.12,7138.62");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2888.28,7143.39");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3314.17,7121.06");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3668.20,7192.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4077.52,7239.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5110.72,7227.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5767.88,7054.55");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=181.70,6923.88");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1690.03,6958.22");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2060.58,7000.32");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3603.22,6817.60");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3958.57,6889.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4646.37,6946.51");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5386.14,6818.92");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=466.67,6695.10");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1282.68,6731.68");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1629.25,6613.51");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2572.18,6684.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3198.70,6757.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4031.59,6533.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5954.12,6615.38");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=98.19,6427.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=903.05,6463.88");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2054.78,6475.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3354.96,6422.56");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3752.82,6320.66");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4554.69,6491.14");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5168.20,6503.43");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5669.94,6389.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=523.54,6225.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1184.11,6134.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1863.31,6106.57");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3145.62,6116.48");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4335.06,6078.62");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4716.37,6028.94");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5079.16,6161.79");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1409.64,5852.78");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2447.73,5948.03");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3628.97,5988.30");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3989.22,5960.42");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5448.61,5833.21");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5796.77,6005.09");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=220.21,5618.58");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=799.27,5621.09");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2902.28,5600.19");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3466.71,5600.83");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5095.15,5701.18");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1063.15,5343.34");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1952.86,5517.12");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3807.13,5303.64");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4187.08,5449.87");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4669.86,5503.41");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5336.03,5429.15");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5812.77,5382.85");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=162.50,5084.34");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1362.91,5154.68");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1878.95,5079.22");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2516.62,5162.01");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3193.22,5215.93");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4709.13,5107.03");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=644.40,4891.00");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1074.68,4856.59");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2444.09,4815.44");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3336.26,4836.23");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3862.63,4890.04");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4193.47,5019.53");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5324.68,4919.54");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5682.00,5010.60");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1555.01,4589.58");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2122.52,4575.11");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2917.27,4665.64");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4270.46,4573.79");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4668.44,4669.94");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5088.73,4589.17");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5957.44,4757.07");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=59.24,4415.87");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=429.55,4394.83");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1046.08,4426.32");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3382.03,4373.95");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3733.48,4344.30");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5444.56,4367.17");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=714.90,4071.00");
    }

}

