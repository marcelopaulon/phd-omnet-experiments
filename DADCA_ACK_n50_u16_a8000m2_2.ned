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

network DADCA_ACK_n50_u16_a8000m2_2
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
        @display("i=misc/sensor;p=228.71,7845.41");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=641.99,7936.18");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1089.82,7902.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1641.82,7972.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2037.92,7952.73");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2718.70,7791.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3299.83,7952.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4016.58,7793.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4518.53,7863.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5140.99,7914.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=507.03,7558.97");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=948.85,7509.07");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2442.48,7539.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3082.87,7664.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3726.26,7522.75");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4742.74,7577.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5806.06,7708.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=74.17,7305.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1919.57,7408.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4290.06,7469.08");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5272.18,7385.94");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=664.59,7190.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1614.22,7205.66");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2493.67,7118.26");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2981.38,7212.14");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3397.64,7185.90");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3784.09,7081.05");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5913.06,7219.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=393.50,6860.36");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1059.98,6876.62");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1753.80,6856.11");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2077.78,7001.12");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4728.19,6998.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5074.16,6779.52");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5506.62,6946.39");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=753.16,6626.33");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2101.91,6605.67");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3106.18,6558.23");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3633.41,6572.35");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4340.73,6715.43");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=515.23,6365.08");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1255.57,6490.04");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1609.23,6496.93");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3944.21,6392.58");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4651.27,6392.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5270.14,6462.39");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=896.96,6204.28");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1460.43,6174.00");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2146.00,6186.05");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2775.93,6155.19");
    }

}

