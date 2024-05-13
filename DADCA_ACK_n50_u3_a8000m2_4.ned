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

network DADCA_ACK_n50_u3_a8000m2_4
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
        @display("i=misc/sensor;p=943.88,7779.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1471.08,7871.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1831.03,7862.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2519.05,7754.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3398.20,7997.49");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3794.49,7876.94");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4224.52,7975.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5398.09,7855.51");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=485.20,7704.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1334.25,7530.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3117.99,7743.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4766.07,7705.25");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5747.58,7605.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1719.01,7406.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2329.02,7425.31");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2873.74,7463.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3431.14,7315.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4043.65,7347.21");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5197.73,7466.69");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1055.87,7250.91");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2785.69,7051.69");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4504.89,7238.48");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5107.40,7109.87");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=417.96,6935.86");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1647.14,6982.42");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2236.32,6972.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3898.52,6835.83");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4230.47,6961.83");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4870.93,6797.13");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5536.55,6962.61");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5.95,6691.76");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=929.08,6742.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1344.69,6653.89");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2026.61,6560.68");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2539.28,6596.73");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3470.29,6757.02");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4372.46,6602.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5144.46,6539.39");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=59.23,6322.86");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3133.74,6296.04");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3555.49,6315.47");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4570.30,6300.61");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=459.45,6260.87");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1178.47,6112.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1812.59,6239.82");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2571.39,6098.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4091.94,6066.84");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5233.22,6131.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5800.68,6200.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=231.70,5938.03");
    }

}

