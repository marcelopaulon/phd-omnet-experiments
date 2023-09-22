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

network DADCA_ACK_n100_a8000m2_4
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
        @display("i=misc/sensor;p=144.96,7981.37");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=264.86,7989.92");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=369.46,7960.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=699.15,7978.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=809.57,7978.07");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=917.05,7978.24");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=72.79,7937.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=198.61,7926.51");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=557.87,7941.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=630.95,7940.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=320.33,7896.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=454.05,7899.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=607.04,7876.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=788.25,7906.49");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=142.73,7861.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=235.24,7845.47");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=322.53,7834.11");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=717.35,7861.38");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=67.95,7784.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=381.05,7805.63");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=448.75,7810.12");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=510.27,7817.29");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=648.52,7817.67");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=141.85,7782.02");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=246.93,7765.21");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=569.71,7769.34");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=710.05,7755.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=804.10,7751.15");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=204.66,7706.42");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=422.42,7724.92");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=508.61,7725.76");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=865.95,7707.80");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=69.57,7694.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=370.61,7692.27");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=544.81,7675.17");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=629.48,7679.56");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=770.13,7671.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=138.92,7636.93");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=333.63,7612.06");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=462.38,7621.96");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=963.15,7622.87");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=69.46,7583.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=264.48,7603.16");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=403.59,7586.71");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=657.27,7605.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=723.71,7601.03");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=835.47,7596.68");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=106.38,7534.65");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=194.11,7564.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=584.29,7566.72");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=751.18,7530.27");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=883.23,7537.06");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=60.21,7483.65");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=264.71,7518.23");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=333.38,7523.21");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=489.10,7500.39");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=648.11,7514.69");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=819.40,7508.98");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=988.27,7524.33");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=120.10,7450.61");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=184.49,7474.25");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=286.30,7457.01");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=396.61,7476.44");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=560.54,7476.48");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=752.72,7456.05");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=919.65,7468.16");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=342.15,7400.72");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=491.52,7415.55");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=691.94,7430.31");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=833.24,7416.35");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=6.83,7388.63");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=109.35,7371.64");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=196.95,7381.22");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=259.71,7381.84");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=433.27,7383.11");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=554.74,7382.47");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=616.87,7363.16");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=698.82,7356.65");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=755.57,7390.39");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=940.27,7368.78");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=158.42,7330.92");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=244.47,7316.97");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=351.84,7309.99");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=531.27,7314.39");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=776.19,7310.99");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=987.76,7313.40");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=65.25,7284.98");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=470.86,7301.08");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=612.40,7297.70");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=882.46,7302.68");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=154.45,7265.31");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=318.11,7239.81");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=412.14,7239.03");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=504.50,7250.08");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=562.20,7225.55");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=705.62,7255.51");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=925.88,7257.51");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=40.32,7190.18");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=253.01,7218.20");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=446.54,7184.19");
    }

}

