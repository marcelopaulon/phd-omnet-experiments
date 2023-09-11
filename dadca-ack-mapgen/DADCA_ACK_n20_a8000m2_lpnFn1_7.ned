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

network DADCA_ACK_n20_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=70.32,28.07");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=112.33,162.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=170.94,138.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=178.47,72.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=256.83,35.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=293.97,112.75");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=273.39,202.03");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=235.51,283.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=224.64,350.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=239.48,412.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=254.91,478.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=351.32,457.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=360.50,553.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=388.35,636.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=237.19,595.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=261.84,661.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=302.16,582.93");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=325.98,673.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=384.05,721.10");
    }

}

