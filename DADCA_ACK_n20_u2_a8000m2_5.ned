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

network DADCA_ACK_n20_u2_a8000m2_5
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
        @display("i=misc/sensor;p=760.97,7887.05");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1860.17,7997.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2384.65,7840.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2810.33,7822.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3484.72,7760.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4348.88,7962.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=342.18,7734.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1065.14,7522.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1439.53,7521.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3193.50,7519.54");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4610.36,7545.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4933.16,7724.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5277.90,7650.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=233.77,7342.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1880.85,7478.93");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2254.87,7365.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2787.14,7297.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3889.14,7472.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=849.86,7142.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1583.74,7183.82");
    }

}

