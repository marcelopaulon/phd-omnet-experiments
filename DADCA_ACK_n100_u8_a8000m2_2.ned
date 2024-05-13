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

network DADCA_ACK_n100_u8_a8000m2_2
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
        @display("i=misc/sensor;p=228.79,7878.80");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=616.00,7790.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1314.34,7865.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2488.98,7974.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2874.63,7966.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4334.59,7916.45");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4920.91,7807.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5534.83,7907.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1612.39,7535.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1977.69,7712.82");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3122.74,7681.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3727.08,7598.91");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=68.05,7441.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=410.48,7261.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=787.66,7476.93");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1169.21,7356.38");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2415.72,7323.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3350.71,7398.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4302.76,7319.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4685.80,7459.84");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5091.90,7451.93");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5599.60,7327.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1706.70,7154.99");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2114.31,7049.46");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2859.47,7252.33");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3165.78,7029.48");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3987.76,7120.47");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4787.90,7076.43");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=435.31,6867.50");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1082.63,6892.27");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1528.55,6851.19");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2643.71,6794.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3623.66,7007.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5662.04,6885.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=395.16,6516.48");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2022.78,6618.71");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3261.30,6566.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3830.23,6569.71");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4258.69,6745.99");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5077.23,6550.63");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=968.78,6448.62");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1592.91,6488.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4715.99,6400.06");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5910.08,6498.30");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1273.45,6058.93");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1680.59,6145.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2033.68,6238.77");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2767.06,6238.80");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3716.09,6116.16");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4108.94,6171.65");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5058.31,6025.72");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5607.05,6253.83");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=209.87,5984.22");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1948.59,5858.08");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2446.22,5836.37");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3013.70,5951.31");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3516.00,5813.80");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4117.27,5799.55");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=173.64,5589.86");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=806.09,5713.19");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1449.74,5697.37");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2200.11,5531.93");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4554.62,5744.72");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5277.65,5728.58");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5855.45,5608.66");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=952.58,5347.23");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1766.14,5421.27");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2600.34,5278.91");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2998.45,5466.65");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3563.38,5454.14");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3957.51,5475.39");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4828.14,5424.71");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=592.43,5155.94");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1405.03,5061.46");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3932.83,5109.94");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4385.71,5065.43");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5365.32,5105.03");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=63.36,4818.15");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1958.61,4936.42");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2313.71,4786.96");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2712.29,4902.11");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3398.67,4855.31");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5902.83,5016.87");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1049.81,4770.39");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1658.86,4690.00");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3999.95,4617.94");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4789.24,4772.80");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5554.90,4647.92");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=458.82,4534.63");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1030.85,4344.92");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1495.69,4314.22");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1916.63,4422.46");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2838.75,4512.56");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3504.07,4478.27");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4495.47,4522.63");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4916.95,4386.68");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=70.89,4149.49");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=469.71,4044.08");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2835.66,4077.39");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3975.79,4180.37");
    }

}

