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

network DADCA_ACK_n20_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=81.25,188.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=25.86,53.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=172.97,143.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=224.19,107.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=20.00,222.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=70.02,268.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=92.34,332.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=170.15,374.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=237.58,405.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=327.04,436.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=389.56,477.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=465.02,459.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=526.77,472.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=542.26,381.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=479.60,357.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=428.67,319.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=578.59,441.06");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=521.84,296.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=564.26,240.79");
    }

}

