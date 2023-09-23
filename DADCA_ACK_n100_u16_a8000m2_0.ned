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

network DADCA_ACK_n100_u16_a8000m2_0
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
        @display("i=misc/sensor;p=534.97,7977.75");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2179.25,7813.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2607.41,7839.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3017.30,7955.23");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3721.59,7966.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4842.27,7933.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5727.42,7840.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1791.24,7613.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3689.50,7596.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4251.50,7632.42");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5330.92,7673.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=154.46,7423.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=615.51,7272.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=915.80,7467.25");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1294.85,7464.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2279.13,7258.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4446.00,7271.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4795.25,7407.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5177.03,7274.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5813.59,7476.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1460.78,7059.36");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1868.87,7065.04");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3048.70,7192.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3429.81,7104.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3841.78,7126.06");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5948.33,7137.43");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=107.17,7009.68");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=972.04,6973.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2198.58,6891.69");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4817.18,6842.40");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5436.00,6970.03");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=520.01,6676.09");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1178.95,6561.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1829.54,6628.81");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2653.51,6695.29");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3279.88,6548.18");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3727.14,6612.44");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4410.70,6698.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5305.40,6539.89");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5678.11,6638.93");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=676.62,6346.16");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2874.76,6386.61");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4126.50,6484.59");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4737.21,6440.58");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5903.47,6283.54");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1162.16,6078.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1564.52,6040.65");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1938.87,6249.79");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3426.77,6185.48");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4142.85,6083.74");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4625.60,6062.75");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5095.11,6132.21");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5466.49,6171.98");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2409.99,6009.87");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3159.15,5891.72");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3741.99,5947.56");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5502.36,5794.10");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=21.50,5614.96");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=375.55,5740.13");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1162.03,5598.19");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1782.41,5726.23");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3361.43,5567.81");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4910.15,5771.58");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5871.36,5593.71");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=289.08,5336.06");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=650.29,5475.68");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1506.84,5292.16");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1910.10,5307.40");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2234.02,5468.97");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2769.46,5524.52");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3630.71,5285.25");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4021.23,5449.30");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4747.94,5418.65");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5218.45,5456.78");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5665.03,5288.26");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1160.12,5053.93");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2448.35,5141.21");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3184.82,5262.70");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=730.76,4950.54");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1701.00,4934.38");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2273.69,4795.82");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2726.43,4840.82");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3102.73,4798.95");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3989.29,4789.90");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4552.10,5012.58");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4908.73,4866.95");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5261.04,4814.75");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5823.10,4895.40");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=389.31,4678.20");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1165.70,4692.66");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1880.55,4543.68");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3491.68,4624.55");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=657.55,4370.45");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1122.95,4310.37");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2264.77,4417.30");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2632.20,4344.58");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4039.11,4389.54");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4401.99,4527.18");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4971.78,4465.66");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5826.74,4460.06");
    }

}

