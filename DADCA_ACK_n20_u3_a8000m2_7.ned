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

network DADCA_ACK_n20_u3_a8000m2_7
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
        @display("i=misc/sensor;p=2.21,7842.14");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1053.81,7908.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1673.66,7922.67");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2277.10,7822.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2773.35,7975.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3167.15,7813.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3831.01,7946.23");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=921.70,7523.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1408.66,7685.44");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=443.82,7433.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1849.71,7493.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2597.62,7331.79");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3314.10,7381.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3817.71,7326.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4438.58,7471.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=690.89,7113.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1053.48,7113.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1545.33,7017.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2113.23,7140.59");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4226.88,7052.93");
    }

}

