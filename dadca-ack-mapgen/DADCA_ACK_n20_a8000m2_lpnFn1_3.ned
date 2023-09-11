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

network DADCA_ACK_n20_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=126.10,44.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=67.62,25.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=217.56,117.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=202.25,25.46");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=276.61,6.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=307.84,101.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=376.75,79.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=410.12,144.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=500.49,176.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=568.39,216.17");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=613.33,163.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=521.79,273.38");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=463.66,309.42");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=425.36,363.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=364.64,215.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=278.69,197.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=142.24,204.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=120.11,270.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=146.23,325.98");
    }

}

