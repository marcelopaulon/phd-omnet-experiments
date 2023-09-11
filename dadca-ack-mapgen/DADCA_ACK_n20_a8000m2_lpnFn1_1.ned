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

network DADCA_ACK_n20_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=163.44,136.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=205.26,198.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=153.74,265.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=26.86,106.42");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=38.92,231.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=119.65,198.62");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=119.84,21.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=30.90,16.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=262.70,107.54");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=200.29,54.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=336.40,22.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=409.95,82.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=484.10,75.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=484.59,14.34");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=554.80,23.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=585.75,96.27");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=645.91,125.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=582.53,195.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=612.14,248.02");
    }

}

