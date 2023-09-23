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

network DADCA_ACK_n100_u2_a8000m2_4
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
        @display("i=misc/sensor;p=592.97,7868.89");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1839.09,7922.95");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2277.54,7967.50");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3177.93,7978.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5004.11,7957.53");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5620.60,7825.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=44.11,7722.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1272.01,7743.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2790.37,7725.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3607.48,7552.83");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4675.52,7597.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=469.67,7499.88");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=965.56,7348.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1840.68,7493.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2338.80,7436.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3912.73,7301.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5352.81,7396.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=35.09,7244.61");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=460.42,7112.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1485.60,7219.45");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2734.81,7069.09");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3105.20,7144.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4514.14,7047.18");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5258.63,7044.28");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5902.48,7072.29");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1023.11,6989.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2058.56,6899.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3463.05,6913.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4136.32,6923.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4765.23,6767.64");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5617.01,6805.42");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=78.62,6551.29");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=598.83,6559.28");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1208.35,6607.80");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1646.46,6689.65");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2922.36,6654.24");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3604.69,6543.70");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5128.85,6563.69");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1640.70,6295.65");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2612.82,6401.88");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4210.20,6509.94");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5758.84,6455.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=428.81,6231.19");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1237.17,6063.21");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2005.10,6234.40");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2892.75,6150.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3763.81,6021.26");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4802.76,6193.80");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5544.00,6034.93");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=353.57,5777.73");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=941.44,5808.23");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2200.08,5937.87");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3377.89,5877.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1582.35,5643.21");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2140.39,5535.60");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2504.45,5758.89");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4209.49,5691.12");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5049.29,5692.09");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5451.64,5687.22");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5906.02,5689.03");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=166.43,5390.03");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=682.47,5505.50");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1121.28,5399.71");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2824.44,5503.01");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3165.32,5411.05");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3585.60,5443.24");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3955.74,5432.36");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4783.87,5460.97");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=777.82,5156.22");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2389.22,5268.24");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4369.47,5165.53");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4979.27,5091.62");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5406.81,5241.11");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=332.39,4976.51");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1079.83,4830.28");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1605.37,5014.09");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1939.85,4870.23");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2980.23,5028.64");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3289.51,4836.90");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3994.96,4995.59");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=371.43,4621.87");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1506.54,4647.66");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2383.48,4778.88");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3568.27,4609.78");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4535.11,4611.37");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5223.28,4563.19");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=872.37,4508.30");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2851.76,4415.51");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4210.66,4442.94");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=64.93,4269.85");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1182.81,4117.19");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1903.92,4163.18");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3680.45,4268.86");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4936.42,4085.18");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5584.35,4051.80");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=100.69,3812.82");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=475.29,4009.58");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2507.46,4038.62");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2916.69,3998.68");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3408.27,4039.37");
    }

}

