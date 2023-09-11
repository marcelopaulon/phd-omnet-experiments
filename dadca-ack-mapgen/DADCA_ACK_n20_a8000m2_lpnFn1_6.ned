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

network DADCA_ACK_n20_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=130.07,36.92");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=61.54,42.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=222.03,1.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=312.33,13.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=374.69,49.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=445.87,83.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=498.53,129.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=506.30,210.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=566.63,168.99");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=573.03,76.43");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=503.96,39.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=571.43,11.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=654.18,29.10");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=650.98,141.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=741.13,55.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=747.78,146.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=670.95,266.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=588.09,256.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=557.45,350.30");
    }

}

