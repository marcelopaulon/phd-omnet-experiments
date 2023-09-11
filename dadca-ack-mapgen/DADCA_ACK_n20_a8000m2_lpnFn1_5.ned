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

network DADCA_ACK_n20_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=144.29,57.11");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=214.22,34.09");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=214.64,102.24");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=74.84,21.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=297.39,37.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=114.49,182.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=252.76,148.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=177.11,150.33");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=32.77,168.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=4.95,30.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=14.95,105.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=68.45,269.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=130.68,244.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=186.47,220.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=305.05,108.44");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=365.80,160.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=306.07,184.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=376.09,228.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=248.91,222.98");
    }

}

