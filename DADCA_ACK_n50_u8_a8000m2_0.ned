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

network DADCA_ACK_n50_u8_a8000m2_0
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
        @display("i=misc/sensor;p=1077.42,7842.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1679.77,7846.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2031.30,7909.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3501.97,7840.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4442.60,7923.45");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4876.79,7901.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5231.41,7996.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=387.89,7720.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1366.34,7538.04");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1877.99,7525.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2526.96,7695.81");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3059.00,7699.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4048.99,7691.12");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4504.12,7573.06");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4944.55,7553.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5762.37,7708.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=197.65,7345.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3388.39,7503.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5310.41,7504.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5981.05,7344.23");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=856.90,7154.96");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1837.04,7072.49");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2785.19,7083.74");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3193.02,7202.48");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3949.10,7210.91");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4531.90,7159.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4923.37,7137.28");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1218.90,6989.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2210.72,6934.37");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3392.32,6898.82");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3798.79,6884.40");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5367.35,6850.58");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5899.89,6764.05");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=269.13,6680.09");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=678.96,6582.12");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1044.56,6575.12");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2862.26,6555.35");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4499.98,6726.45");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5141.22,6538.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1493.80,6300.96");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1816.42,6509.98");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3757.49,6514.77");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4220.18,6492.63");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5847.76,6415.32");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=413.33,6252.38");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=867.01,6257.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2137.87,6132.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2475.81,6035.99");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3149.40,6217.37");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4662.82,6218.83");
    }

}

