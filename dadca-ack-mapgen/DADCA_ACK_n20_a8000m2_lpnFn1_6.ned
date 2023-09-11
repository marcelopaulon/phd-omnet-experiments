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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=126.70,177.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=210.01,214.47");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=241.58,163.24");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=158.79,117.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=41.47,156.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=75.48,227.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=108.20,318.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=7.97,375.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=34.78,436.21");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=27.71,266.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=186.11,352.39");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=123.33,390.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=249.10,328.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=276.77,237.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=143.55,446.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=221.92,457.67");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=286.25,447.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=167.04,515.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=221.39,566.12");
    }

}

