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
        @display("i=misc/sensor,p=99.89,182.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=48.44,57.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=10.52,112.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=169.52,152.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=46.50,233.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=17.10,179.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=196.34,86.22");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=171.45,14.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=232.22,22.45");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=271.55,172.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=208.21,238.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=110.26,13.85");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=310.10,84.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=370.62,74.29");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=376.05,1.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=460.88,133.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=465.91,203.41");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=528.23,255.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=523.05,162.51");
    }

}

