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

network DADCA_ACK_n20_u4_a8000m2_3
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
        @display("i=misc/sensor;p=1810.84,7906.53");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2893.59,7851.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3240.87,7768.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3693.99,7809.46");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5170.96,7795.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2115.57,7542.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4436.11,7621.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5618.51,7634.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1404.25,7443.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2486.38,7332.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3235.89,7357.24");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3994.20,7467.12");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4760.99,7482.21");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=591.57,7226.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1917.49,7073.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2848.43,7218.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4032.83,7056.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4429.89,7122.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5115.63,7137.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5787.27,7032.62");
    }

}

