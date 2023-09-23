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

network DADCA_ACK_n100_u3_a8000m2_9
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
        @display("i=misc/sensor;p=1770.79,7794.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3198.24,7885.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3769.93,7967.66");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4760.18,7973.78");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5135.77,7888.86");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5780.73,7827.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=400.80,7671.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1062.19,7665.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2070.55,7578.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2710.00,7571.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3692.85,7604.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4319.77,7621.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1441.07,7471.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3218.05,7291.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4911.29,7502.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5304.77,7338.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=171.30,7150.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=527.62,7116.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1009.87,7242.29");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1910.61,7177.14");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2320.74,7181.25");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2712.27,7171.34");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3842.47,7232.17");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4746.43,7081.74");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1354.15,6872.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2106.09,6830.37");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2822.38,6798.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4337.10,6877.38");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5772.62,6932.80");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=259.25,6536.46");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=719.27,6730.43");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1625.89,6636.92");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2383.03,6563.78");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3570.33,6703.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4767.22,6687.80");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5303.80,6665.95");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=835.43,6305.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1209.54,6318.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1979.11,6337.96");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2659.92,6330.14");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4745.16,6302.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3161.66,6177.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3683.33,6098.47");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4294.20,6246.91");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5191.22,6155.35");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5691.09,6124.33");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=279.59,5860.02");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=694.32,5872.17");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1334.09,5990.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1789.04,5984.71");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2327.36,5947.97");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3270.80,5824.41");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3955.78,5855.07");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4797.02,5923.57");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5990.07,5897.28");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1021.64,5710.80");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1743.09,5585.52");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2661.23,5577.65");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4353.53,5717.08");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5149.74,5610.32");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=181.62,5311.41");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1227.51,5298.11");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3238.74,5367.64");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3675.83,5413.37");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4787.52,5476.57");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5529.29,5423.80");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=647.43,5174.08");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2285.33,5252.92");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2920.09,5121.75");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3809.23,5070.32");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4124.47,5257.55");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5135.17,5084.21");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=945.21,4986.53");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1538.55,4962.98");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1868.80,4835.93");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2524.82,4824.01");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3426.23,4800.04");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4708.21,4971.27");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5566.72,4954.18");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=336.44,4667.37");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=893.32,4637.45");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3954.77,4546.35");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4302.10,4645.64");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5346.41,4634.15");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1442.81,4514.89");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2285.70,4502.39");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2875.89,4423.78");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4537.78,4373.17");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4909.58,4511.98");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5639.20,4309.91");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=469.16,4119.97");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3194.47,4253.42");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4073.75,4101.85");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4951.93,4040.76");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=701.49,3811.67");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1103.85,3981.49");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1448.13,3825.70");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1779.23,4021.56");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2934.16,4013.35");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3556.59,3844.63");
    }

}

