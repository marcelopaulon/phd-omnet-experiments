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
        @display("i=misc/sensor,p=52.27,25.95");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=114.86,159.13");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=177.18,176.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=251.66,190.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=267.41,268.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=199.82,258.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=9.71,244.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=62.69,280.26");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=102.02,227.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=181.19,63.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=258.16,113.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=329.00,136.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=340.51,50.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=421.36,101.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=212.41,8.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=496.61,81.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=487.39,3.15");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=544.87,27.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=490.32,154.20");
    }

}

