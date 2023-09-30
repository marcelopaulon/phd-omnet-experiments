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

network DADCA_ACK_n100_u16_a8000m2_1
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
        @display("i=misc/sensor;p=1790.36,7769.27");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3482.02,7952.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4852.66,7824.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5902.60,7977.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=810.47,7693.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1416.76,7714.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2397.46,7731.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2882.14,7685.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3537.78,7547.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3955.20,7542.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4419.23,7715.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5426.05,7683.21");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=399.41,7476.70");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1117.54,7356.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1692.71,7427.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2083.82,7464.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5364.24,7307.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5730.87,7284.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=882.64,7057.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1387.64,7116.34");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2342.36,7188.54");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3092.49,7140.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3641.84,7211.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3997.72,7083.69");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4770.41,7182.08");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=508.99,6952.89");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1660.62,6797.87");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2731.83,6811.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4235.43,6816.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5674.12,6932.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=166.79,6562.26");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1185.05,6761.57");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2334.21,6638.26");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3377.00,6634.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3797.82,6542.24");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4606.97,6717.18");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5141.83,6557.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=946.34,6298.99");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1367.45,6395.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4261.09,6437.61");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5642.10,6480.46");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=576.64,6229.15");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1744.92,6193.70");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2124.28,6217.64");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2716.80,6144.05");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3230.18,6109.43");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3730.19,6141.10");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4250.91,6027.00");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4945.00,6115.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5462.67,6084.88");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5973.46,6140.64");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3.79,5940.16");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1274.61,5916.73");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5208.99,5792.02");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5823.69,5808.69");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=546.22,5741.62");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=949.45,5674.44");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1623.58,5697.76");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2008.26,5761.35");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2536.38,5605.21");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3143.96,5711.42");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3652.63,5711.10");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4807.74,5675.76");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1368.36,5441.53");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2882.07,5421.84");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4375.20,5488.74");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5796.90,5285.55");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=888.43,5198.75");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1550.54,5043.56");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1888.74,5164.87");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2350.08,5277.36");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2766.87,5047.73");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3245.22,5047.66");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3974.07,5183.21");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4985.20,5267.98");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5430.41,5078.73");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=522.90,4822.63");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3642.95,5018.77");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4214.42,4815.19");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4706.25,4886.70");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=6.49,4684.84");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1004.13,4748.45");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2466.68,4716.87");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3119.33,4687.39");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3741.97,4615.88");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5299.25,4715.56");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5653.56,4602.19");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=64.01,4329.27");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2281.73,4339.29");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2977.23,4335.67");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3411.49,4471.86");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4103.82,4459.01");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4777.93,4409.18");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=766.67,4287.11");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1510.97,4256.88");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3687.98,4159.89");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4110.21,4045.32");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4476.70,4155.74");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5054.11,4177.94");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5802.11,4050.80");
    }

}

