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

network DADCA_ACK_n100_a8000m2_3
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
        @display("i=misc/sensor;p=97.51,7960.17");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=178.76,7993.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=423.93,7964.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=557.97,7962.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=630.11,7981.49");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=715.48,7974.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=912.46,7967.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=271.96,7952.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=370.74,7934.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=825.09,7942.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=47.27,7904.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=197.38,7911.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=479.65,7903.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=650.44,7899.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=995.86,7892.65");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=118.86,7861.53");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=267.66,7833.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=340.11,7852.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=447.07,7830.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=510.96,7833.11");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=582.43,7863.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=754.30,7863.16");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=894.35,7857.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=964.33,7839.95");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=40.98,7805.21");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=195.23,7819.77");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=676.41,7812.44");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=870.28,7785.68");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=158.11,7753.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=280.11,7747.49");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=365.05,7770.53");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=508.83,7770.70");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=582.09,7751.54");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=707.15,7754.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=802.47,7783.75");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=357.41,7709.16");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=434.80,7716.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=953.61,7704.44");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=36.14,7696.35");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=247.53,7672.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=422.72,7655.65");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=656.27,7685.20");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=765.14,7667.69");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=833.50,7677.03");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=143.05,7640.12");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=369.17,7624.04");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=484.82,7640.59");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=546.08,7643.74");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=926.75,7638.75");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=996.52,7642.42");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3.54,7605.80");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=200.70,7582.39");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=442.38,7587.49");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=696.85,7584.07");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=838.74,7592.11");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=7.17,7525.93");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=81.85,7558.62");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=140.41,7528.20");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=282.95,7561.94");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=611.26,7535.02");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=784.72,7539.65");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=915.20,7540.98");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=257.80,7485.94");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=383.68,7510.28");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=524.09,7518.07");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=0.27,7442.02");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=60.47,7474.56");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=167.65,7463.78");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=384.81,7443.57");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=444.22,7459.57");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=544.35,7453.69");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=664.00,7473.61");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=749.36,7459.50");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=205.99,7409.34");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=306.88,7430.93");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=486.62,7413.86");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=594.22,7417.33");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=788.17,7408.02");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=845.61,7431.63");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=933.10,7409.12");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=94.10,7394.17");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=170.88,7358.35");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=558.12,7363.56");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=707.66,7375.56");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=33.02,7350.93");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=257.56,7317.04");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=393.45,7322.68");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=477.30,7352.20");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=624.96,7329.83");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=776.16,7323.13");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=889.49,7340.41");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=965.09,7335.45");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=90.10,7292.70");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=179.81,7276.06");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=511.12,7299.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=576.98,7269.99");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=676.14,7272.81");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=933.53,7268.10");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=11.75,7252.35");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=140.48,7225.16");
    }

}

