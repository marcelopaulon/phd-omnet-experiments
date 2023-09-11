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
        @display("i=misc/sensor,p=36.16,65.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=98.27,19.67");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=192.31,31.23");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=201.15,107.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=275.34,55.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=352.92,51.82");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=280.45,135.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=435.28,36.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=386.40,1.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=366.30,143.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=404.55,93.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=417.65,224.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=444.63,151.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=541.30,24.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=606.07,15.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=554.84,100.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=471.77,92.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=557.21,169.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=611.26,132.29");
    }

}

