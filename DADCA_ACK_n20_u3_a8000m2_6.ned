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

network DADCA_ACK_n20_u3_a8000m2_6
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
        @display("i=misc/sensor;p=322.98,7979.41");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=693.69,7983.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2001.41,7754.49");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4315.09,7777.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5076.52,7833.56");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=266.87,7558.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=775.66,7627.35");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1103.56,7752.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1444.00,7576.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3304.98,7702.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3861.94,7516.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=560.11,7351.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1908.55,7313.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2409.28,7364.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2942.64,7325.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4634.09,7450.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5345.57,7441.31");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5.04,7177.82");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1076.11,7065.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3520.76,7179.75");
    }

}

