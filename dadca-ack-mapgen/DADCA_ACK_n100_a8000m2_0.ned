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

network DADCA_ACK_n100_a8000m2_0
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
        @display("i=misc/sensor;p=92.33,7963.93");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=273.46,7965.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=359.85,7985.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=694.55,7965.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=780.55,7964.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=863.35,7975.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=422.96,7942.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=505.47,7952.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=923.49,7920.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=240.24,7892.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=362.51,7896.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=543.57,7901.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=653.22,7897.86");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=991.75,7873.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=59.29,7859.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=413.49,7842.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=472.46,7862.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=593.06,7837.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=728.27,7852.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=863.28,7867.78");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=145.24,7793.46");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=242.60,7798.63");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=356.49,7795.10");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=452.47,7793.53");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=652.06,7806.72");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=804.54,7790.85");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=909.57,7809.69");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=977.55,7799.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=26.23,7782.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=197.02,7754.57");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=532.75,7781.98");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=593.70,7773.39");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=68.07,7706.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=139.72,7702.72");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=481.96,7721.72");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=705.52,7736.53");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=799.87,7703.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=867.48,7711.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=10.23,7658.23");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=209.54,7683.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=283.51,7685.23");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=395.65,7686.79");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=581.32,7692.66");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=667.75,7662.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=942.53,7682.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=147.36,7618.64");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=208.39,7616.10");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=340.47,7631.67");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=566.27,7621.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=753.79,7642.05");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=863.89,7641.37");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=48.38,7603.23");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=273.07,7571.65");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=436.53,7608.43");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=628.86,7590.40");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=921.72,7575.66");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=999.77,7609.60");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=210.85,7540.23");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=401.15,7526.42");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=576.19,7535.70");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=693.80,7555.77");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=779.01,7567.68");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=841.89,7531.05");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=7.68,7516.40");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=146.03,7524.46");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=250.00,7484.67");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=325.52,7515.57");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=503.51,7524.18");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=658.95,7494.44");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=746.71,7487.21");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=927.93,7513.95");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=993.23,7500.11");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=87.61,7446.40");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=179.22,7472.47");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=622.67,7440.60");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=871.05,7473.07");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=168.71,7405.19");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=399.62,7426.08");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=516.16,7419.82");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=699.86,7436.46");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=779.34,7421.59");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=947.75,7405.22");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=16.49,7370.81");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=88.07,7361.90");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=231.71,7371.13");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=329.78,7354.81");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=397.48,7354.89");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=597.07,7368.93");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=675.86,7375.65");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=860.71,7367.22");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=476.74,7342.47");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=747.91,7347.51");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=818.15,7309.91");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=11.79,7287.03");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=100.86,7269.43");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=186.10,7287.20");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=536.86,7272.45");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=693.39,7307.90");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=880.35,7273.21");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=43.39,7234.64");
    }

}

