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
        @display("i=misc/sensor;p=41.24,117.89");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3.77,58.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=103.90,219.47");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=147.73,304.15");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=94.33,375.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=158.77,441.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=188.74,497.03");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=109.57,512.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=106.84,584.10");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=38.13,554.84");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=169.97,571.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=97.82,647.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=27.83,674.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=120.45,706.33");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=34.86,458.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=159.98,368.98");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=30.42,323.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=38.03,228.07");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=233.40,147.19");
    }

}

