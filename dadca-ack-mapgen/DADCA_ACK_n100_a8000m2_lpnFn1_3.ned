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

network DADCA_ACK_n100_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=163.41,110.66");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=89.69,246.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=144.55,187.60");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=215.61,188.39");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=157.80,293.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=163.00,370.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=152.17,432.94");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=239.15,396.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=315.59,412.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=332.57,336.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=311.03,252.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=360.58,212.74");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=326.07,143.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=60.21,148.90");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=47.49,60.89");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=126.82,6.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=239.63,121.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=249.27,30.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=466.27,287.07");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=541.22,282.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=558.31,379.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=528.51,474.50");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=598.51,317.07");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=565.70,553.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=510.44,586.74");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=596.09,614.93");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=660.91,636.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=449.23,596.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=426.38,653.47");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=335.29,614.90");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=293.38,557.31");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=228.21,587.03");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=219.67,651.17");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=162.03,589.40");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=216.92,517.70");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=281.52,470.49");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=224.48,281.45");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=467.43,142.24");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=419.76,62.69");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=369.96,22.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=578.01,76.44");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=516.22,80.65");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=538.95,190.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=406.04,152.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=462.52,213.76");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=390.03,274.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=105.67,518.52");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=42.26,503.39");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=50.89,600.55");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=436.05,388.31");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=494.00,355.81");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=608.47,158.65");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=605.48,12.53");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=724.79,110.64");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=664.06,51.11");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=753.45,22.63");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=799.54,105.41");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=873.24,119.96");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=904.19,30.56");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=981.19,49.87");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=850.66,191.39");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=942.60,124.58");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=919.52,204.57");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=962.18,261.97");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1038.50,268.78");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1071.80,191.33");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1010.05,150.81");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1167.47,209.08");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1137.35,280.28");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1195.93,263.86");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1122.44,341.12");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1040.61,393.79");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=950.82,399.80");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=883.70,324.02");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=885.36,412.22");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=850.45,261.26");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=821.13,332.81");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=789.67,395.21");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=736.51,312.26");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=676.81,357.47");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=627.15,414.16");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=682.30,444.40");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=644.66,501.11");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=405.68,441.49");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=362.09,528.29");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=314.08,48.30");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=4.85,251.12");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=29.01,312.09");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=88.82,326.42");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=96.02,403.30");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=270.45,336.08");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=0.00,158.72");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=21.88,386.77");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=459.89,497.54");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=338.22,715.57");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=490.04,423.93");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=652.66,703.17");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=645.58,795.42");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=703.41,831.06");
    }

}

