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

network DADCA_ACK_n100_u2_a8000m2_1
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
        @display("i=misc/sensor;p=2728.05,7995.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3250.29,7808.57");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=455.76,7545.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1093.39,7571.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1881.56,7643.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2303.81,7558.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2967.87,7539.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3971.20,7725.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4604.92,7536.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3609.33,7429.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3995.30,7288.52");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4973.41,7424.98");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5695.83,7445.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=131.81,7248.13");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=586.19,7061.93");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=941.87,7200.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1466.37,7225.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1887.31,7100.11");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2289.28,7141.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3011.50,7165.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3453.67,7082.31");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4392.85,7225.11");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1617.34,6811.62");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2508.07,6774.28");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3945.88,6863.42");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4882.50,6996.64");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5232.85,6949.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5815.54,6812.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=581.26,6704.94");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1100.39,6629.29");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2171.70,6612.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4302.18,6584.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4678.50,6690.54");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5171.99,6603.23");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=454.20,6292.93");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1628.48,6308.22");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2687.54,6440.95");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3056.16,6286.13");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3501.93,6414.49");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4022.83,6332.04");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4894.26,6314.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5755.69,6466.44");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1070.83,6204.09");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2239.01,6143.03");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5250.57,6252.00");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2485.86,5775.77");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2849.16,5871.21");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3428.18,5871.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3845.31,5995.03");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4229.18,5999.40");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4657.54,6018.17");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5061.15,5795.48");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5441.76,5913.72");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5824.91,5951.75");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=211.00,5654.46");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=717.03,5764.31");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1128.03,5613.80");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1916.67,5735.79");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3635.09,5558.29");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4167.59,5652.37");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5956.20,5580.52");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=715.92,5297.91");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3141.53,5335.30");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4510.27,5366.86");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4874.79,5417.08");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5448.74,5403.23");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=302.46,5218.53");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1664.80,5274.93");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2296.38,5240.00");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3452.88,5085.27");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4141.95,5109.97");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5769.98,5210.48");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=632.69,4947.17");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1127.98,4958.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1683.38,4787.95");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2060.49,4905.52");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2848.49,4991.63");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4766.05,4905.73");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5168.42,4930.79");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1416.05,4541.97");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3335.67,4640.15");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3696.29,4731.83");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4239.53,4539.08");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5433.05,4663.44");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5910.64,4721.25");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=246.20,4499.54");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1002.86,4507.47");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2068.41,4507.31");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2735.78,4403.50");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3905.47,4411.22");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4901.27,4314.59");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=732.11,4271.43");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1777.40,4169.81");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3396.82,4271.55");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4635.39,4064.55");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5623.78,4166.94");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=379.06,4017.26");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=917.42,3930.01");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1356.83,3986.40");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2344.97,3927.22");
    }

}

